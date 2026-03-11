// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// attrs_convert.cpp — Attribute and metadata conversion helpers.
//
// Translates non-geometry data (object names, display colours, layers,
// materials, document metadata, unit system) between an openNURBS ONX_Model
// and an OpenCASCADE XDE document (TDocStd_Document + XCAF attributes).
//
// Phase-2 items implemented here:
//   P1  Object name      ON_3dmObjectAttributes::m_name  ↔  TDataStd_Name
//   P1  Object colour    ON_3dmObjectAttributes::m_color ↔  XCAFDoc_ColorTool
//   P1  Layer name/color ON_Layer                        ↔  XCAFDoc_LayerTool
//   P2  Material         ON_Material                     ↔  XCAFDoc_MaterialTool
//   P2  Document author  ON_3dmRevisionHistory::m_sCreatedBy ↔ TDataStd_Name
//   P2  Unit system      ON_3dmUnitsAndTolerances         ↔  XCAFDoc_LengthUnit
// Phase-3 items implemented here:
//   P3  Instance defs / blocks  ON_InstanceDefinition + ON_InstanceRef
//                               ↔  XCAFDoc_ShapeTool assembly + component

#include "open2open/attrs_convert.h"
#include "open2open/brep_convert.h"
#include "open2open/mesh_convert.h"

#include "opennurbs.h"

// OCCT — attribute framework
#include <TDocStd_Document.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_TreeNode.hxx>

// OCCT — XCAF document tools
#include <XCAFDoc.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_MaterialTool.hxx>
#include <XCAFDoc_LengthUnit.hxx>
#include <XCAFDoc_ViewTool.hxx>
#include <XCAFDoc_View.hxx>
#include <XCAFView_Object.hxx>
#include <XCAFView_ProjectionType.hxx>

// OCCT — colour
#include <Quantity_Color.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_HAsciiString.hxx>

// OCCT — shape
#include <TopoDS_Shape.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>

// OCCT — topology iteration
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>

// OCCT — mesh (for SubD fallback / mesh objects)
#include <Poly_Triangulation.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

// OCCT — transforms (for instance placement)
#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <TopLoc_Location.hxx>
#include <XCAFDoc_Location.hxx>

// OCCT — text annotation / notes
#include <XCAFDoc_NotesTool.hxx>
#include <XCAFDoc_NoteComment.hxx>
#include <XCAFDoc_Note.hxx>
#include <TDataStd_Comment.hxx>
#include <TDataStd_Real.hxx>
#include <TDF_ChildIterator.hxx>

// OCCT — GD&T / dimensioning
#include <XCAFDoc_DimTolTool.hxx>
#include <XCAFDoc_DimTol.hxx>

#include <map>
#include <set>
#include <string>
#include <vector>
#include <array>
#include <functional>
#include <cmath>

namespace open2open {

// ---------------------------------------------------------------------------
// String helpers
// ---------------------------------------------------------------------------

/// Convert ON_wString to TCollection_ExtendedString via UTF-8 round-trip.
static TCollection_ExtendedString ONwToExt(const ON_wString& s)
{
    // ON_String assigned from ON_wString produces a UTF-8 encoded narrow string.
    ON_String utf8;
    utf8 = s; // triggers UTF-32→UTF-8 conversion on Linux
    if (utf8.IsEmpty())
        return TCollection_ExtendedString();
    // TCollection_ExtendedString(char*, isMultiByte=true) reads UTF-8.
    return TCollection_ExtendedString(utf8.Array(), /*isMultiByte=*/Standard_True);
}

/// Convert TCollection_ExtendedString to ON_wString via UTF-8 round-trip.
static ON_wString ExtToONw(const TCollection_ExtendedString& ext)
{
    // ToUTF8CString writes into a caller-supplied buffer (it does NOT allocate).
    // Allocate UTF-8 buffer: worst case 4 bytes per UTF-16 code unit, plus null.
    const int len = ext.Length();
    if (len == 0)
        return ON_wString();
    std::vector<char> buf(static_cast<size_t>(len) * 4 + 1);
    Standard_PCharacter p = buf.data();
    ext.ToUTF8CString(p);
    return ON_wString(buf.data()); // ON_wString(char*) reads UTF-8 on Linux
}

// ---------------------------------------------------------------------------
// Colour helpers
// ---------------------------------------------------------------------------

/// Convert ON_Color to Quantity_ColorRGBA (alpha: 1 = opaque).
static Quantity_ColorRGBA ONColorToRGBA(ON_Color c)
{
    // ON_Color alpha: 0=opaque, 255=fully transparent
    const float alpha = 1.0f - (float)c.Alpha() / 255.0f;
    return Quantity_ColorRGBA(
        Quantity_Color((float)c.FractionRed(),
                       (float)c.FractionGreen(),
                       (float)c.FractionBlue(),
                       Quantity_TOC_RGB),
        alpha);
}

/// Convert Quantity_ColorRGBA to ON_Color.
static ON_Color RGBAToONColor(const Quantity_ColorRGBA& rgba)
{
    const Quantity_Color& rgb = rgba.GetRGB();
    // alpha: 0=opaque, 255=transparent
    const int alpha = (int)((1.0f - rgba.Alpha()) * 255.0f + 0.5f);
    return ON_Color((int)(rgb.Red()   * 255.0 + 0.5),
                    (int)(rgb.Green() * 255.0 + 0.5),
                    (int)(rgb.Blue()  * 255.0 + 0.5),
                    alpha);
}

// ---------------------------------------------------------------------------
// Transform helpers (P3 — instance definitions)
// ---------------------------------------------------------------------------

/// Convert an ON_Xform (row-major 4×4) to a gp_Trsf (3×4 affine).
/// Only the 3×4 upper block is used; the last row must be [0,0,0,1].
static gp_Trsf ONXformToGpTrsf(const ON_Xform& x)
{
    gp_Trsf trsf;
    trsf.SetValues(x.m_xform[0][0], x.m_xform[0][1], x.m_xform[0][2], x.m_xform[0][3],
                   x.m_xform[1][0], x.m_xform[1][1], x.m_xform[1][2], x.m_xform[1][3],
                   x.m_xform[2][0], x.m_xform[2][1], x.m_xform[2][2], x.m_xform[2][3]);
    return trsf;
}

/// Convert a gp_Trsf (3×4 affine, 1-indexed) to an ON_Xform (4×4 row-major).
static ON_Xform GpTrsfToONXform(const gp_Trsf& trsf)
{
    ON_Xform x;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            x.m_xform[r][c] = trsf.Value(r + 1, c + 1);
    x.m_xform[3][0] = x.m_xform[3][1] = x.m_xform[3][2] = 0.0;
    x.m_xform[3][3] = 1.0;
    return x;
}

// ---------------------------------------------------------------------------
// ON_UUID comparison for std::map
// ---------------------------------------------------------------------------
struct UuidLess {
    bool operator()(const ON_UUID& a, const ON_UUID& b) const {
        return std::memcmp(&a, &b, sizeof(ON_UUID)) < 0;
    }
};

/// Return scale-to-metres factor for an openNURBS length unit system.
static double ONUnitScaleToMetres(ON::LengthUnitSystem us)
{
    // ON::UnitScale(from, to) returns multiplier from→to.
    return ON::UnitScale(us, ON::LengthUnitSystem::Meters);
}

/// Best-fit openNURBS unit system for a given metres scale factor.
static ON::LengthUnitSystem MetresScaleToONUnit(double scaleToM)
{
    struct Entry { ON::LengthUnitSystem us; double scale; };
    static const Entry kTable[] = {
        { ON::LengthUnitSystem::Millimeters,  1e-3  },
        { ON::LengthUnitSystem::Centimeters,  1e-2  },
        { ON::LengthUnitSystem::Meters,       1.0   },
        { ON::LengthUnitSystem::Inches,       0.0254 },
        { ON::LengthUnitSystem::Feet,         0.3048 },
        { ON::LengthUnitSystem::Yards,        0.9144 },
        { ON::LengthUnitSystem::Kilometers,   1000.0 },
        { ON::LengthUnitSystem::Miles,        1609.344 },
        { ON::LengthUnitSystem::Microns,      1e-6  },
        { ON::LengthUnitSystem::Nanometers,   1e-9  },
    };
    ON::LengthUnitSystem best = ON::LengthUnitSystem::Millimeters;
    double bestErr = 1e30;
    for (const auto& e : kTable) {
        const double err = std::abs(e.scale - scaleToM);
        if (err < bestErr) { bestErr = err; best = e.us; }
    }
    return best;
}

// ---------------------------------------------------------------------------
// CreateXCAFDocument
// ---------------------------------------------------------------------------

Handle(TDocStd_Document) CreateXCAFDocument()
{
    // Create the document in-memory; format tag "open2open" is arbitrary
    // (only matters when persisting).
    Handle(TDocStd_Document) doc = new TDocStd_Document("open2open");
    // Initialise XCAF document-tool chain (ShapeTool, ColorTool, LayerTool…)
    XCAFDoc_DocumentTool::Set(doc->Main());
    return doc;
}

// ---------------------------------------------------------------------------
// ONX_ModelToXCAFDoc
// ---------------------------------------------------------------------------

Handle(TDocStd_Document) ONX_ModelToXCAFDoc(const ONX_Model& model, double tol)
{
    Handle(TDocStd_Document) doc = CreateXCAFDocument();

    Handle(XCAFDoc_ShapeTool)    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool)    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    Handle(XCAFDoc_LayerTool)    layerTool = XCAFDoc_DocumentTool::LayerTool(doc->Main());
    Handle(XCAFDoc_MaterialTool) matTool   = XCAFDoc_DocumentTool::MaterialTool(doc->Main());

    // -----------------------------------------------------------------------
    // P2: Document metadata — created-by on the doc root label
    // -----------------------------------------------------------------------
    {
        const ON_wString& creator =
            model.m_properties.m_RevisionHistory.m_sCreatedBy;
        if (creator.IsNotEmpty()) {
            TDataStd_Name::Set(doc->Main(), ONwToExt(creator));
        }
    }

    // -----------------------------------------------------------------------
    // P2: Unit system — XCAFDoc_LengthUnit scale-to-metres on doc root
    // -----------------------------------------------------------------------
    {
        const ON::LengthUnitSystem us =
            model.m_settings.m_ModelUnitsAndTolerances.m_unit_system.UnitSystem();
        const double scale = ONUnitScaleToMetres(us);
        XCAFDoc_DocumentTool::SetLengthUnit(doc, scale);
    }

    // -----------------------------------------------------------------------
    // P1: Layer table — each ON_Layer becomes an XCAF layer entry.
    // The layer colour is stored as a colour on the layer's own XCAF label.
    // Map: ON layer index → XCAF layer label
    // -----------------------------------------------------------------------
    std::map<int, TDF_Label> layerLabelMap;
    {
        // Build a map from ON_UUID → layer name to resolve parent names
        // for encoding hierarchical names as "ParentName/ChildName".
        std::map<ON_UUID, ON_wString, UuidLess> layerNameById;
        {
            ONX_ModelComponentIterator lit2(model, ON_ModelComponent::Type::Layer);
            for (const ON_ModelComponent* mc2 = lit2.FirstComponent();
                 mc2 != nullptr;
                 mc2 = lit2.NextComponent())
            {
                const ON_Layer* l2 = ON_Layer::Cast(mc2);
                if (l2) layerNameById[l2->Id()] = l2->Name();
            }
        }

        ONX_ModelComponentIterator lit(model, ON_ModelComponent::Type::Layer);
        for (const ON_ModelComponent* mc = lit.FirstComponent();
             mc != nullptr;
             mc = lit.NextComponent())
        {
            const ON_Layer* layer = ON_Layer::Cast(mc);
            if (!layer) continue;

            const int idx = layer->Index();
            const ON_wString& name = layer->Name();

            // Build fully-qualified name: "Parent/Child" for nested layers
            ON_wString fullName = name.IsNotEmpty() ? name : ON_wString(L"Layer");
            const ON_UUID parentId = layer->ParentLayerId();
            if (parentId != ON_nil_uuid) {
                auto pit = layerNameById.find(parentId);
                if (pit != layerNameById.end() && pit->second.IsNotEmpty()) {
                    ON_wString qualified = pit->second;
                    qualified += L"/";
                    qualified += fullName;
                    fullName = qualified;
                }
            }

            TCollection_ExtendedString extName = ONwToExt(fullName);
            TDF_Label layerLabel = layerTool->AddLayer(extName);
            layerLabelMap[idx] = layerLabel;

            // Store layer colour on the layer's label as a generic colour.
            const ON_Color lc = layer->Color();
            if (lc != ON_Color::UnsetColor) {
                Quantity_ColorRGBA qc = ONColorToRGBA(lc);
                colorTool->SetColor(layerLabel, qc, XCAFDoc_ColorGen);
            }
        }
    }

    // -----------------------------------------------------------------------
    // P2: Material table — each ON_Material becomes an XCAF material entry.
    // Map: ON material index → XCAF material label
    // -----------------------------------------------------------------------
    std::map<int, TDF_Label> matLabelMap;
    std::map<int, ON_Color>  matDiffuseMap; // stored separately for shape colour
    {
        ONX_ModelComponentIterator mit(model,
                                       ON_ModelComponent::Type::RenderMaterial);
        for (const ON_ModelComponent* mc = mit.FirstComponent();
             mc != nullptr;
             mc = mit.NextComponent())
        {
            const ON_Material* mat = ON_Material::Cast(mc);
            if (!mat) continue;

            const int idx = mat->Index();
            const ON_wString& mname = mat->Name();

            Handle(TCollection_HAsciiString) hName =
                new TCollection_HAsciiString(
                    mname.IsNotEmpty()
                        ? ON_String(mname).Array()
                        : "Material");

            Handle(TCollection_HAsciiString) hDesc =
                new TCollection_HAsciiString("");
            Handle(TCollection_HAsciiString) hDensName =
                new TCollection_HAsciiString("kg/m3");
            Handle(TCollection_HAsciiString) hDensType =
                new TCollection_HAsciiString("POSITIVE_RATIO_MEASURE");

            TDF_Label matLabel = matTool->AddMaterial(
                hName, hDesc,
                /*density=*/0.0,
                hDensName, hDensType);

            matLabelMap[idx]   = matLabel;
            matDiffuseMap[idx] = mat->Diffuse();

            // P4: Embedded texture maps — store texture file references as
            // TDataStd_Comment attributes on the material label.
            // Format: "TEXTURE:<type_int>:<full_path>"
            // Multiple textures → multiple attributes on successive sub-labels.
            for (int ti = 0; ti < mat->m_textures.Count(); ++ti) {
                const ON_Texture& tex = mat->m_textures[ti];
                const ON_wString& fp = tex.m_image_file_reference.FullPath();
                if (fp.IsEmpty()) continue;
                ON_String fpUtf8;
                fpUtf8 = fp;
                ON_String entry;
                entry.Format("TEXTURE:%u:%s",
                             static_cast<unsigned>(tex.m_type),
                             fpUtf8.Array());
                // Create a child label per texture
                TDF_Label texLabel = matLabel.NewChild();
                TDataStd_Comment::Set(texLabel,
                    TCollection_ExtendedString(entry.Array(), Standard_True));
            }
        }
    }

    // -----------------------------------------------------------------------
    // P1/P2: Geometry objects — first pass: brep/extrusion shapes.
    // Also records ON_UUID → TDF_Label for instance-definition assembly.
    // -----------------------------------------------------------------------
    std::map<ON_UUID, TDF_Label, UuidLess> uuidToLabel; // P3: geometry UUID → XCAF label

    ONX_ModelComponentIterator it(model, ON_ModelComponent::Type::ModelGeometry);
    for (const ON_ModelComponent* mc = it.FirstComponent();
         mc != nullptr;
         mc = it.NextComponent())
    {
        const ON_ModelGeometryComponent* mgc =
            ON_ModelGeometryComponent::Cast(mc);
        if (!mgc) continue;

        const ON_Geometry* geom = mgc->Geometry(nullptr);
        if (!geom) continue;

        const ON_3dmObjectAttributes* attrs = mgc->Attributes(nullptr);

        // Skip instance references — handled in the P3 pass below
        if (geom->ObjectType() == ON::instance_reference)
            continue;

        // P4: Text annotations & GD&T Dimensions — annotation_object covers both
        if (geom->ObjectType() == ON::annotation_object) {
            const ON_Annotation* ann = ON_Annotation::Cast(geom);
            if (!ann) continue;

            // GD&T: if it's a dimension sub-class, store via DimTolTool
            const ON_Dimension* dim = dynamic_cast<const ON_Dimension*>(ann);
            if (dim) {
                Handle(XCAFDoc_DimTolTool) dimTolTool =
                    XCAFDoc_DocumentTool::DimTolTool(doc->Main());
                Handle(TCollection_HAsciiString) hName =
                    new TCollection_HAsciiString();
                Handle(TCollection_HAsciiString) hDesc =
                    new TCollection_HAsciiString();
                if (attrs && attrs->m_name.IsNotEmpty()) {
                    ON_String utf8;
                    utf8 = attrs->m_name;
                    hName = new TCollection_HAsciiString(utf8.Array());
                }
                Handle(TColStd_HArray1OfReal) vals =
                    new TColStd_HArray1OfReal(1, 1);
                vals->SetValue(1, 0.0);
                TDF_Label dimLabel =
                    dimTolTool->AddDimTol(0, vals, hName, hDesc);
                if (!dimLabel.IsNull() && attrs && attrs->m_name.IsNotEmpty())
                    TDataStd_Name::Set(dimLabel, ONwToExt(attrs->m_name));
                continue;
            }

            // General annotation (ON_Text, ON_Leader, …): store as comment note
            const ON_wString txt = ann->PlainText();
            if (txt.IsEmpty()) continue;
            Handle(XCAFDoc_NotesTool) notesTool =
                XCAFDoc_DocumentTool::NotesTool(doc->Main());
            TCollection_ExtendedString extTxt = ONwToExt(txt);
            Handle(XCAFDoc_Note) note =
                notesTool->CreateComment(
                    TCollection_ExtendedString(""),
                    TCollection_ExtendedString(""),
                    extTxt);
            if (!note.IsNull()) {
                if (attrs && attrs->m_name.IsNotEmpty())
                    TDataStd_Name::Set(note->Label(), ONwToExt(attrs->m_name));
                const ON_3dPoint& org = ann->Plane().origin;
                ON_String coordStr;
                coordStr.Format("%.17g %.17g %.17g", org.x, org.y, org.z);
                TDataStd_Comment::Set(note->Label(),
                    TCollection_ExtendedString(coordStr.Array(), Standard_True));
            }
            continue;
        }

        // Convert geometry to TopoDS_Shape
        TopoDS_Shape shape;
        if (geom->ObjectType() == ON::brep_object) {
            const ON_Brep* brep = ON_Brep::Cast(geom);
            if (!brep) continue;
            shape = ON_BrepToOCCT(*brep, tol);
        } else if (geom->ObjectType() == ON::extrusion_object) {
            const ON_Extrusion* ext = ON_Extrusion::Cast(geom);
            if (!ext) continue;
            ON_Brep* b = ext->BrepForm();
            if (!b) continue;
            shape = ON_BrepToOCCT(*b, tol);
            delete b;
        } else if (geom->ObjectType() == ON::mesh_object) {
            // P4: Mesh objects — convert via ON_MeshToOCCT + attach as face
            const ON_Mesh* mesh = ON_Mesh::Cast(geom);
            if (!mesh) continue;
            Handle(Poly_Triangulation) tri = ON_MeshToOCCT(*mesh);
            if (tri.IsNull()) continue;
            // Wrap in a compound face so ShapeTool can manage it
            TopoDS_Compound comp;
            BRep_Builder bb;
            bb.MakeCompound(comp);
            // Add vertices of the mesh as a compound shape
            const int nv = mesh->m_V.Count();
            const bool useDbl = (mesh->m_dV.Count() == nv);
            for (int vi = 0; vi < nv; ++vi) {
                gp_Pnt pt;
                if (useDbl) {
                    const ON_3dPoint& p = mesh->m_dV[vi];
                    pt = gp_Pnt(p.x, p.y, p.z);
                } else {
                    const ON_3fPoint& p = mesh->m_V[vi];
                    pt = gp_Pnt((double)p.x, (double)p.y, (double)p.z);
                }
                TopoDS_Vertex vtx = BRepBuilderAPI_MakeVertex(pt);
                bb.Add(comp, vtx);
            }
            shape = comp;
        } else if (geom->ObjectType() == ON::subd_object) {
            // P4: SubD → mesh fallback via control-net mesh
            const ON_SubD* subd = ON_SubD::Cast(geom);
            if (!subd) continue;
            ON_Mesh meshStorage;
            ON_Mesh* result = subd->GetControlNetMesh(
                &meshStorage,
                ON_SubDGetControlNetMeshPriority::Geometry);
            if (!result || result->m_V.Count() == 0)
                continue;
            // Store as vertex compound
            TopoDS_Compound comp;
            BRep_Builder bb;
            bb.MakeCompound(comp);
            const int snv = result->m_V.Count();
            const bool sUseDbl = (result->m_dV.Count() == snv);
            for (int vi = 0; vi < snv; ++vi) {
                gp_Pnt pt;
                if (sUseDbl) {
                    const ON_3dPoint& p = result->m_dV[vi];
                    pt = gp_Pnt(p.x, p.y, p.z);
                } else {
                    const ON_3fPoint& p = result->m_V[vi];
                    pt = gp_Pnt((double)p.x, (double)p.y, (double)p.z);
                }
                TopoDS_Vertex vtx = BRepBuilderAPI_MakeVertex(pt);
                bb.Add(comp, vtx);
            }
            shape = comp;
        } else if (geom->ObjectType() == ON::pointset_object) {
            // P4: Point clouds — compound of OCCT vertices
            const ON_PointCloud* pc = ON_PointCloud::Cast(geom);
            if (!pc) continue;
            const int np = pc->PointCount();
            if (np <= 0) continue;
            TopoDS_Compound comp;
            BRep_Builder bb;
            bb.MakeCompound(comp);
            for (int pi = 0; pi < np; ++pi) {
                const ON_3dPoint& p = pc->m_P[pi];
                TopoDS_Vertex vtx =
                    BRepBuilderAPI_MakeVertex(gp_Pnt(p.x, p.y, p.z));
                bb.Add(comp, vtx);
            }
            shape = comp;
        } else {
            continue;
        }
        if (shape.IsNull()) continue;

        // Add to XCAF shape tool.
        // Use makeAssembly=false for compounds (vertex clouds, mesh fallbacks)
        // to prevent XCAF from exploding the compound into sub-components.
        const bool isCompound = (shape.ShapeType() == TopAbs_COMPOUND);
        TDF_Label label = shapeTool->AddShape(
            shape, /*makeAssembly=*/isCompound ? Standard_False : Standard_True);
        if (label.IsNull()) continue;

        // Record UUID → label for instance-definition assembly pass
        uuidToLabel[mc->Id()] = label;

        // P1: Object name
        if (attrs && attrs->m_name.IsNotEmpty()) {
            TDataStd_Name::Set(label, ONwToExt(attrs->m_name));
        }

        // P1: Object display colour (only when colour-from-object)
        if (attrs &&
            attrs->ColorSource() == ON::color_from_object)
        {
            Quantity_ColorRGBA qc = ONColorToRGBA(attrs->m_color);
            colorTool->SetColor(label, qc, XCAFDoc_ColorGen);
        }

        // P2: Material diffuse colour (overrides object colour if both set)
        if (attrs && attrs->m_material_index >= 0) {
            auto it2 = matDiffuseMap.find(attrs->m_material_index);
            if (it2 != matDiffuseMap.end() &&
                it2->second != ON_Color::UnsetColor)
            {
                Quantity_ColorRGBA qc = ONColorToRGBA(it2->second);
                colorTool->SetColor(label, qc, XCAFDoc_ColorSurf);
            }
        }

        // P1: Layer assignment
        if (attrs) {
            auto lit2 = layerLabelMap.find(attrs->m_layer_index);
            if (lit2 != layerLabelMap.end()) {
                TCollection_ExtendedString layerName;
                layerTool->GetLayer(lit2->second, layerName);
                if (!layerName.IsEmpty()) {
                    layerTool->SetLayer(shape, layerName,
                                        /*shapeInOneLayer=*/Standard_True);
                }
            }
        }

        // P2: Material assignment
        if (attrs && attrs->m_material_index >= 0) {
            auto mit2 = matLabelMap.find(attrs->m_material_index);
            if (mit2 != matLabelMap.end()) {
                matTool->SetMaterial(label, mit2->second);
            }
        }
    }

    // -----------------------------------------------------------------------
    // P3: Instance definitions — create XCAF "product" assemblies.
    // Each ON_InstanceDefinition becomes an XCAF assembly label whose sub-
    // components are the geometry shapes referenced by the idef's member list.
    // Map: idef ON_UUID → XCAF assembly label.
    // -----------------------------------------------------------------------
    std::map<ON_UUID, TDF_Label, UuidLess> idefUuidToLabel;

    {
        ONX_ModelComponentIterator idit(model,
            ON_ModelComponent::Type::InstanceDefinition);
        for (const ON_ModelComponent* mc = idit.FirstComponent();
             mc != nullptr;
             mc = idit.NextComponent())
        {
            const ON_InstanceDefinition* idef =
                ON_InstanceDefinition::Cast(mc);
            if (!idef) continue;

            // Create a new empty XCAF assembly label for this block definition
            TDF_Label asmLabel = shapeTool->NewShape();
            if (asmLabel.IsNull()) continue;

            // Set the name on the assembly label
            if (idef->Name().IsNotEmpty())
                TDataStd_Name::Set(asmLabel, ONwToExt(idef->Name()));

            // Add each member geometry as a sub-component with identity location
            const ON_SimpleArray<ON_UUID>& members =
                idef->InstanceGeometryIdList();
            for (int i = 0; i < members.Count(); ++i) {
                auto found = uuidToLabel.find(members[i]);
                if (found == uuidToLabel.end())
                    continue;
                shapeTool->AddComponent(asmLabel, found->second,
                                        TopLoc_Location()); // identity
            }

            idefUuidToLabel[mc->Id()] = asmLabel;
        }
    }

    // -----------------------------------------------------------------------
    // P3: Instance references — create XCAF components with locations.
    // Each ON_InstanceRef becomes an XCAF component in a top-level "scene"
    // assembly, referencing the corresponding instance-definition label.
    // If no instance refs exist the scene assembly is not created.
    // -----------------------------------------------------------------------
    {
        // Collect all instance refs first so we can decide whether to create
        // the scene assembly
        struct IRefInfo {
            TDF_Label     idefLabel;
            TopLoc_Location loc;
            ON_wString    name;
        };
        std::vector<IRefInfo> irefInfos;

        ONX_ModelComponentIterator geoIt(model,
            ON_ModelComponent::Type::ModelGeometry);
        for (const ON_ModelComponent* mc = geoIt.FirstComponent();
             mc != nullptr;
             mc = geoIt.NextComponent())
        {
            const ON_ModelGeometryComponent* mgc =
                ON_ModelGeometryComponent::Cast(mc);
            if (!mgc) continue;
            const ON_Geometry* geom = mgc->Geometry(nullptr);
            if (!geom || geom->ObjectType() != ON::instance_reference)
                continue;
            const ON_InstanceRef* iref = ON_InstanceRef::Cast(geom);
            if (!iref) continue;

            // Find the XCAF assembly for the referenced instance definition
            auto found = idefUuidToLabel.find(
                iref->m_instance_definition_uuid);
            if (found == idefUuidToLabel.end())
                continue;

            // Convert ON_Xform → gp_Trsf → TopLoc_Location
            gp_Trsf trsf = ONXformToGpTrsf(iref->m_xform);
            TopLoc_Location loc(trsf);

            const ON_3dmObjectAttributes* attrs = mgc->Attributes(nullptr);
            ON_wString name;
            if (attrs) name = attrs->m_name;

            irefInfos.push_back({found->second, loc, name});
        }

        if (!irefInfos.empty()) {
            // Create a top-level "Scene" compound to hold the instances
            TDF_Label sceneLabel = shapeTool->NewShape();
            TDataStd_Name::Set(sceneLabel,
                               TCollection_ExtendedString("Scene"));

            for (const auto& info : irefInfos) {
                TDF_Label compLabel =
                    shapeTool->AddComponent(sceneLabel,
                                            info.idefLabel,
                                            info.loc);
                if (!compLabel.IsNull() && info.name.IsNotEmpty())
                    TDataStd_Name::Set(compLabel, ONwToExt(info.name));
            }
        }
    }

    // -----------------------------------------------------------------------
    // P3: Named views — each ON_3dmView becomes an XCAF view entry.
    // Stores name, projection type, camera location/direction/up.
    // -----------------------------------------------------------------------
    {
        Handle(XCAFDoc_ViewTool) viewTool =
            XCAFDoc_DocumentTool::ViewTool(doc->Main());

        const int nViews = model.m_settings.m_named_views.Count();
        for (int vi = 0; vi < nViews; ++vi) {
            const ON_3dmView& v = model.m_settings.m_named_views[vi];
            const ON_Viewport& vp = v.m_vp;

            // Skip degenerate viewports
            if (!vp.IsValidCamera()) continue;

            TDF_Label viewLabel = viewTool->AddView();
            if (viewLabel.IsNull()) continue;

            Handle(XCAFView_Object) vo = new XCAFView_Object();

            // Name
            Handle(TCollection_HAsciiString) hName =
                new TCollection_HAsciiString(
                    v.m_name.IsNotEmpty()
                        ? ON_String(v.m_name).Array()
                        : "View");
            vo->SetName(hName);

            // Projection type
            vo->SetType(vp.IsPerspectiveProjection()
                ? XCAFView_ProjectionType_Central
                : XCAFView_ProjectionType_Parallel);

            // Camera location (projection point in parallel projection = target)
            const ON_3dPoint loc = vp.CameraLocation();
            vo->SetProjectionPoint(gp_Pnt(loc.x, loc.y, loc.z));

            // View direction
            const ON_3dVector dir = vp.CameraDirection();
            if (dir.IsNotZero()) {
                ON_3dVector d = dir;
                d.Unitize();
                vo->SetViewDirection(gp_Dir(d.x, d.y, d.z));
            }

            // Up direction
            const ON_3dVector up = vp.CameraUp();
            if (up.IsNotZero()) {
                ON_3dVector u = up;
                u.Unitize();
                vo->SetUpDirection(gp_Dir(u.x, u.y, u.z));
            }

            // Window size (frustum width/height at near plane)
            vo->SetWindowHorizontalSize(vp.FrustumWidth());
            vo->SetWindowVerticalSize(vp.FrustumHeight());

            // Zoom factor: use 1/frustum_height as a normalised scale
            const double fh = vp.FrustumHeight();
            if (fh > 0.0)
                vo->SetZoomFactor(1.0 / fh);

            Handle(XCAFDoc_View) viewAttr = XCAFDoc_View::Set(viewLabel);
            viewAttr->SetObject(vo);

            // Name the label
            if (v.m_name.IsNotEmpty())
                TDataStd_Name::Set(viewLabel, ONwToExt(v.m_name));

            // Associate with all free shapes
            TDF_LabelSequence shapeLabels;
            shapeTool->GetFreeShapes(shapeLabels);
            viewTool->SetView(shapeLabels, TDF_LabelSequence(), viewLabel);
        }
    }

    return doc;
}

// ---------------------------------------------------------------------------
// XCAFDocToONX_Model
// ---------------------------------------------------------------------------

bool XCAFDocToONX_Model(const Handle(TDocStd_Document)& doc,
                        ONX_Model&                      model,
                        double                          tol)
{
    if (doc.IsNull()) return false;

    model.Reset();

    Handle(XCAFDoc_ShapeTool)    shapeTool = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool)    colorTool = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    Handle(XCAFDoc_LayerTool)    layerTool = XCAFDoc_DocumentTool::LayerTool(doc->Main());
    Handle(XCAFDoc_MaterialTool) matTool   = XCAFDoc_DocumentTool::MaterialTool(doc->Main());

    // -----------------------------------------------------------------------
    // P2: Document metadata — created-by from doc root label
    // -----------------------------------------------------------------------
    {
        Handle(TDataStd_Name) nameAttr;
        if (doc->Main().FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
            model.m_properties.m_RevisionHistory.m_sCreatedBy =
                ExtToONw(nameAttr->Get());
        }
    }

    // -----------------------------------------------------------------------
    // P2: Unit system — read XCAFDoc_LengthUnit from doc root
    // -----------------------------------------------------------------------
    {
        Standard_Real scaleToM = 1.0;
        if (XCAFDoc_DocumentTool::GetLengthUnit(doc, scaleToM) && scaleToM > 0.0) {
            ON::LengthUnitSystem us = MetresScaleToONUnit(scaleToM);
            model.m_settings.m_ModelUnitsAndTolerances.m_unit_system =
                ON_UnitSystem(us);
        }
    }

    // -----------------------------------------------------------------------
    // P1/P3: Layer table — each XCAF layer label → ON_Layer.
    // Hierarchical names encoded as "Parent/Child" are decoded to set
    // ParentLayerId on the child ON_Layer.
    // Two-pass: first add all layers, then fix parent links using UUID map.
    // Map: layer label tag → ON layer index
    // -----------------------------------------------------------------------
    std::map<Standard_Integer, int> layerTagToIndex;
    {
        TDF_LabelSequence layerLabels;
        layerTool->GetLayerLabels(layerLabels);

        // Pass 1: add all layers with their leaf name; record fqName → UUID.
        std::map<std::string, ON_UUID, std::less<std::string>> fqNameToUuid;

        for (Standard_Integer i = 1; i <= layerLabels.Length(); ++i) {
            const TDF_Label& ll = layerLabels.Value(i);

            TCollection_ExtendedString layerName;
            layerTool->GetLayer(ll, layerName);
            ON_wString fullName = layerName.IsEmpty()
                ? ON_wString(L"Layer") : ExtToONw(layerName);

            // Leaf name = part after last "/"
            ON_wString leafName = fullName;
            const int slashPos = fullName.ReverseFind(L'/');
            if (slashPos >= 0)
                leafName = fullName.Mid(slashPos + 1);

            ON_Layer* onLayer = new ON_Layer();
            onLayer->SetName(leafName.IsNotEmpty()
                ? (const wchar_t*)leafName : L"Layer");

            // Layer colour
            Quantity_ColorRGBA qc;
            if (colorTool->GetColor(ll, XCAFDoc_ColorGen, qc))
                onLayer->m_color = RGBAToONColor(qc);

            ON_ModelComponentReference ref =
                model.AddManagedModelComponent(onLayer, false);
            const int layerIdx = ref.ModelComponentIndex();
            layerTagToIndex[ll.Tag()] = layerIdx;

            // Record fqName → UUID for parent-linking
            ON_String utf8;
            utf8 = fullName;
            const ON_Layer* added = ON_Layer::Cast(ref.ModelComponent());
            if (added)
                fqNameToUuid[std::string(utf8.Array())] = added->Id();
        }

        // Pass 2: set ParentLayerId on child layers.
        for (Standard_Integer i = 1; i <= layerLabels.Length(); ++i) {
            const TDF_Label& ll = layerLabels.Value(i);
            TCollection_ExtendedString layerName;
            layerTool->GetLayer(ll, layerName);
            if (layerName.IsEmpty()) continue;

            ON_wString fullName = ExtToONw(layerName);
            const int slashPos = fullName.ReverseFind(L'/');
            if (slashPos < 0) continue; // top-level

            ON_wString parentFull = fullName.Left(slashPos);
            ON_String parentUtf8;
            parentUtf8 = parentFull;
            auto pit = fqNameToUuid.find(std::string(parentUtf8.Array()));
            if (pit == fqNameToUuid.end()) continue;

            const int childIdx = layerTagToIndex.at(ll.Tag());
            ON_ModelComponentReference childRef =
                model.ComponentFromIndex(
                    ON_ModelComponent::Type::Layer, childIdx);
            const ON_Layer* childLayer = ON_Layer::Cast(childRef.ModelComponent());
            if (!childLayer) continue;

            // Clone with parent set, replace using AddManagedModelComponent
            // with the same UUID (conflict resolution will match by UUID)
            ON_Layer* withParent = new ON_Layer(*childLayer);
            withParent->SetParentLayerId(pit->second);
            model.AddManagedModelComponent(withParent, true);
        }
    }

    // -----------------------------------------------------------------------
    // P2: Material table — each XCAF material label → ON_Material
    // Map: material label tag → ON material index
    // -----------------------------------------------------------------------
    std::map<Standard_Integer, int> matTagToIndex;
    {
        TDF_LabelSequence matLabels;
        matTool->GetMaterialLabels(matLabels);
        for (Standard_Integer i = 1; i <= matLabels.Length(); ++i) {
            const TDF_Label& ml = matLabels.Value(i);

            Handle(TCollection_HAsciiString) hName, hDesc, hDensName, hDensType;
            Standard_Real density = 0.0;
            if (!XCAFDoc_MaterialTool::GetMaterial(
                    ml, hName, hDesc, density, hDensName, hDensType))
                continue;

            ON_Material mat;
            if (!hName.IsNull() && !hName->IsEmpty()) {
                mat.SetName(ON_wString(hName->ToCString()));
            }

            // P4: Restore texture file references stored as TDataStd_Comment
            // on child labels of the material label.
            for (TDF_ChildIterator ci(ml); ci.More(); ci.Next()) {
                const TDF_Label& texLbl = ci.Value();
                Handle(TDataStd_Comment) cmt;
                if (!texLbl.FindAttribute(TDataStd_Comment::GetID(), cmt))
                    continue;
                ON_wString entry = ExtToONw(cmt->Get());
                // Format: "TEXTURE:<type_int>:<path>"
                if (entry.Left(8) != L"TEXTURE:") continue;
                const int firstColon = entry.Find(L':', 8);
                if (firstColon < 0) continue;
                ON_wString typeStr = entry.Mid(8, firstColon - 8);
                ON_wString pathStr = entry.Mid(firstColon + 1);
                if (pathStr.IsEmpty()) continue;

                unsigned typeUint = 0;
                {
                    ON_String ts;
                    ts = typeStr;
                    std::sscanf(ts.Array(), "%u", &typeUint);
                }

                ON_Texture tex;
                tex.m_type = ON_Texture::TypeFromUnsigned(typeUint);
                tex.m_image_file_reference =
                    ON_FileReference::CreateFromFullPath(
                        static_cast<const wchar_t*>(pathStr),
                        false, false);
                mat.AddTexture(tex);
            }

            const int matIdx = model.AddManagedModelComponent(
                new ON_Material(mat), false).ModelComponentIndex();
            if (matIdx >= 0)
                matTagToIndex[ml.Tag()] = matIdx;
        }
    }

    // -----------------------------------------------------------------------
    // P1/P2/P3: Walk ALL shape labels in the document (free + nested).
    //
    // Two passes:
    //   Pass 1: collect all "leaf" (simple, non-assembly) shapes as breps,
    //           including those that are components of assemblies.
    //           Builds labelTag → geomUuid for use in assembly construction.
    //   Pass 2: for each free assembly, create ON_InstanceDefinition + refs.
    // -----------------------------------------------------------------------
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);

    // P3: label-tag → ON_UUID for brep shapes added to the model
    std::map<Standard_Integer, ON_UUID> labelTagToGeomUuid;
    bool anyOK = false;

    // Helper lambda: add one simple-shape label as a brep to the model.
    // Skips labels already processed (idempotent via labelTagToGeomUuid).
    auto addSimpleShape = [&](const TDF_Label& label) {
        Standard_Integer tag = label.Tag();
        if (labelTagToGeomUuid.count(tag))
            return; // already added

        const TopoDS_Shape shape = shapeTool->GetShape(label);
        if (shape.IsNull()) return;

        // P4: Point cloud — detect a compound of vertices
        if (shape.ShapeType() == TopAbs_COMPOUND) {
            // Count vertices vs. other types
            int nVtx = 0, nOther = 0;
            TopExp_Explorer ex(shape, TopAbs_VERTEX, TopAbs_EDGE);
            for (; ex.More(); ex.Next()) ++nVtx;
            TopExp_Explorer ex2(shape, TopAbs_EDGE);
            for (; ex2.More(); ex2.Next()) ++nOther;
            if (nVtx > 0 && nOther == 0) {
                // Reconstruct as ON_PointCloud
                ON_PointCloud* pc = new ON_PointCloud();
                pc->m_P.Reserve(nVtx);
                TopExp_Explorer exv(shape, TopAbs_VERTEX, TopAbs_EDGE);
                for (; exv.More(); exv.Next()) {
                    const gp_Pnt& p =
                        BRep_Tool::Pnt(TopoDS::Vertex(exv.Current()));
                    pc->m_P.Append(ON_3dPoint(p.X(), p.Y(), p.Z()));
                }
                ON_3dmObjectAttributes* pcAttrs = new ON_3dmObjectAttributes();
                {
                    Handle(TDataStd_Name) na;
                    if (label.FindAttribute(TDataStd_Name::GetID(), na))
                        pcAttrs->m_name = ExtToONw(na->Get());
                }
                ON_ModelComponentReference ref2 =
                    model.AddManagedModelGeometryComponent(pc, pcAttrs);
                anyOK = true;
                labelTagToGeomUuid[tag] = ref2.ModelComponent()->Id();
                return;
            }
        }

        ON_Brep* brep = new ON_Brep();
        if (!OCCTToON_Brep(shape, *brep, tol)) {
            delete brep;
            return;
        }
        if (!brep->IsValid() || brep->m_F.Count() == 0) {
            delete brep;
            return;
        }

        // Build per-object attributes
        ON_3dmObjectAttributes* attrs = new ON_3dmObjectAttributes();

        // P1: Name
        {
            Handle(TDataStd_Name) nameAttr;
            if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr))
                attrs->m_name = ExtToONw(nameAttr->Get());
        }

        // P1: Object colour (XCAFDoc_ColorGen preferred, then XCAFDoc_ColorSurf)
        {
            Quantity_ColorRGBA qc;
            if (colorTool->GetColor(label, XCAFDoc_ColorGen, qc) ||
                colorTool->GetColor(label, XCAFDoc_ColorSurf, qc))
            {
                attrs->m_color = RGBAToONColor(qc);
                attrs->SetColorSource(ON::color_from_object);
            }
        }

        // P1: Layer assignment
        {
            Handle(TColStd_HSequenceOfExtendedString) layerNames =
                layerTool->GetLayers(shape);
            if (!layerNames.IsNull() && !layerNames->IsEmpty()) {
                const TCollection_ExtendedString& lname = layerNames->Value(1);
                TDF_LabelSequence lls;
                layerTool->GetLayerLabels(lls);
                for (Standard_Integer j = 1; j <= lls.Length(); ++j) {
                    TCollection_ExtendedString n;
                    if (layerTool->GetLayer(lls.Value(j), n) && n == lname) {
                        auto found = layerTagToIndex.find(lls.Value(j).Tag());
                        if (found != layerTagToIndex.end())
                            attrs->m_layer_index = found->second;
                        break;
                    }
                }
            }
        }

        // P2: Material
        {
            Handle(TDataStd_TreeNode) matNode;
            if (label.FindAttribute(XCAFDoc::MaterialRefGUID(), matNode) &&
                matNode->HasFather())
            {
                TDF_Label matLabel = matNode->Father()->Label();
                auto found = matTagToIndex.find(matLabel.Tag());
                if (found != matTagToIndex.end())
                    attrs->m_material_index = found->second;
            }
        }

        ON_ModelComponentReference ref =
            model.AddManagedModelGeometryComponent(brep, attrs);
        anyOK = true;
        labelTagToGeomUuid[tag] = ref.ModelComponent()->Id();
    };

    // Recursive helper: collect all leaf simple-shape labels in sub-tree.
    // (forward declare the variable then assign the lambda so it can self-recurse)
    std::function<void(const TDF_Label&)> collectAndAddSimple;
    collectAndAddSimple = [&](const TDF_Label& label) {
        if (XCAFDoc_ShapeTool::IsReference(label)) {
            TDF_Label ref;
            if (XCAFDoc_ShapeTool::GetReferredShape(label, ref))
                collectAndAddSimple(ref);
            return;
        }
        if (XCAFDoc_ShapeTool::IsAssembly(label)) {
            TDF_LabelSequence comps;
            XCAFDoc_ShapeTool::GetComponents(label, comps,
                                              /*sub=*/Standard_False);
            if (comps.Length() > 0) {
                for (Standard_Integer k = 1; k <= comps.Length(); ++k)
                    collectAndAddSimple(comps.Value(k));
                return;
            }
            // Assembly with no TDF components: may be a monolithic compound
            // (e.g. a point-cloud vertex compound). Fall through to addSimpleShape.
        }
        // Simple shape: add it to the model
        addSimpleShape(label);
    };

    // Pass 1: add all leaf shapes (free simple + nested inside assemblies)
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i)
        collectAndAddSimple(freeShapes.Value(i));

    // -----------------------------------------------------------------------
    // P3: Assemblies → ON_InstanceDefinition + ON_InstanceRef
    //
    // Algorithm uses ALL shape labels (not just free ones), in two sub-passes:
    //
    // Sub-pass A: "block definition" assemblies — assemblies whose direct
    //   components refer only to simple shapes already in labelTagToGeomUuid.
    //   → Create one ON_InstanceDefinition per such assembly.
    //   → Populate asmTagToIdefUuid for sub-pass B.
    //
    // Sub-pass B: "scene" assemblies — assemblies whose components refer to
    //   other assemblies (block definitions from sub-pass A).
    //   → Create one ON_InstanceRef per such component.
    //
    // Both sub-passes iterate ALL shape labels so non-free assemblies (e.g.
    // a block definition that is itself used as a component) are handled.
    // -----------------------------------------------------------------------

    // Collect ALL shape labels (free + non-free)
    TDF_LabelSequence allShapes;
    shapeTool->GetShapes(allShapes);

    // asmTag → ON_UUID of the corresponding ON_InstanceDefinition (sub-pass A)
    std::map<Standard_Integer, ON_UUID> asmTagToIdefUuid;

    // Sub-pass A: block definition assemblies
    for (Standard_Integer i = 1; i <= allShapes.Length(); ++i) {
        const TDF_Label& asmLabel = allShapes.Value(i);
        if (!XCAFDoc_ShapeTool::IsAssembly(asmLabel))
            continue;
        // Skip if already processed
        if (asmTagToIdefUuid.count(asmLabel.Tag()))
            continue;

        TDF_LabelSequence components;
        XCAFDoc_ShapeTool::GetComponents(asmLabel, components,
                                          /*getsubchilds=*/Standard_False);
        if (components.IsEmpty())
            continue;

        // Check that ALL direct referred shapes are simple (in labelTagToGeomUuid)
        bool allSimple = true;
        for (Standard_Integer j = 1; j <= components.Length(); ++j) {
            TDF_Label refLbl;
            if (!XCAFDoc_ShapeTool::GetReferredShape(
                    components.Value(j), refLbl)) {
                allSimple = false; break;
            }
            if (!labelTagToGeomUuid.count(refLbl.Tag())) {
                allSimple = false; break;
            }
        }
        if (!allSimple)
            continue;

        // Build member UUID list (unique per referred shape)
        ON_SimpleArray<ON_UUID> memberUuids;
        std::set<Standard_Integer> seen;
        for (Standard_Integer j = 1; j <= components.Length(); ++j) {
            TDF_Label refLbl;
            XCAFDoc_ShapeTool::GetReferredShape(components.Value(j), refLbl);
            Standard_Integer tag = refLbl.Tag();
            if (seen.insert(tag).second)
                memberUuids.Append(labelTagToGeomUuid.at(tag));
        }
        if (memberUuids.Count() == 0)
            continue;

        ON_wString idefName;
        {
            Handle(TDataStd_Name) na;
            if (asmLabel.FindAttribute(TDataStd_Name::GetID(), na))
                idefName = ExtToONw(na->Get());
        }

        ON_InstanceDefinition* idef = new ON_InstanceDefinition();
        idef->SetName(idefName.IsNotEmpty() ? (const wchar_t*)idefName : L"Block");
        idef->SetInstanceGeometryIdList(memberUuids);
        ON_ModelComponentReference idefRef =
            model.AddManagedModelComponent(idef, /*resolve conflicts=*/false);
        asmTagToIdefUuid[asmLabel.Tag()] = idefRef.ModelComponent()->Id();
    }

    // Sub-pass B: scene assemblies — components that refer to block definitions
    for (Standard_Integer i = 1; i <= allShapes.Length(); ++i) {
        const TDF_Label& asmLabel = allShapes.Value(i);
        if (!XCAFDoc_ShapeTool::IsAssembly(asmLabel))
            continue;
        // Skip block definitions (already handled) and unrecognised assemblies
        if (asmTagToIdefUuid.count(asmLabel.Tag()))
            continue;

        TDF_LabelSequence components;
        XCAFDoc_ShapeTool::GetComponents(asmLabel, components,
                                          /*getsubchilds=*/Standard_False);

        for (Standard_Integer j = 1; j <= components.Length(); ++j) {
            const TDF_Label& compLabel = components.Value(j);
            TDF_Label refLbl;
            if (!XCAFDoc_ShapeTool::GetReferredShape(compLabel, refLbl))
                continue;
            auto it_idef = asmTagToIdefUuid.find(refLbl.Tag());
            if (it_idef == asmTagToIdefUuid.end())
                continue;

            TopLoc_Location loc = XCAFDoc_ShapeTool::GetLocation(compLabel);
            ON_Xform xform = loc.IsIdentity()
                ? ON_Xform::IdentityTransformation
                : GpTrsfToONXform(loc.Transformation());

            ON_InstanceRef* iref = new ON_InstanceRef();
            iref->m_instance_definition_uuid = it_idef->second;
            iref->m_xform                    = xform;

            ON_3dmObjectAttributes* iattrs = new ON_3dmObjectAttributes();
            Handle(TDataStd_Name) cn;
            if (compLabel.FindAttribute(TDataStd_Name::GetID(), cn))
                iattrs->m_name = ExtToONw(cn->Get());

            model.AddManagedModelGeometryComponent(iref, iattrs);
            anyOK = true;
        }
    }

    // -----------------------------------------------------------------------
    // P3: Named views — read XCAF view labels → ON_3dmView
    // -----------------------------------------------------------------------
    {
        Handle(XCAFDoc_ViewTool) viewTool =
            XCAFDoc_DocumentTool::ViewTool(doc->Main());

        TDF_LabelSequence viewLabels;
        viewTool->GetViewLabels(viewLabels);

        for (Standard_Integer i = 1; i <= viewLabels.Length(); ++i) {
            const TDF_Label& vl = viewLabels.Value(i);

            Handle(XCAFDoc_View) viewAttr;
            if (!vl.FindAttribute(XCAFDoc_View::GetID(), viewAttr))
                continue;
            Handle(XCAFView_Object) vo = viewAttr->GetObject();
            if (vo.IsNull()) continue;

            ON_3dmView onView;

            // Name
            ON_wString viewName;
            if (!vo->Name().IsNull() && !vo->Name()->IsEmpty())
                viewName = ON_wString(vo->Name()->ToCString());
            if (viewName.IsEmpty()) {
                // Fallback: check TDataStd_Name on the label
                Handle(TDataStd_Name) na;
                if (vl.FindAttribute(TDataStd_Name::GetID(), na))
                    viewName = ExtToONw(na->Get());
            }
            if (!viewName.IsEmpty())
                onView.m_name = viewName;

            ON_Viewport& vp = onView.m_vp;

            // Projection type
            const bool isPerspective =
                (vo->Type() == XCAFView_ProjectionType_Central);

            // Camera location
            gp_Pnt projPt = vo->ProjectionPoint();
            vp.SetCameraLocation(
                ON_3dPoint(projPt.X(), projPt.Y(), projPt.Z()));

            // View direction
            gp_Dir viewDir = vo->ViewDirection();
            vp.SetCameraDirection(
                ON_3dVector(viewDir.X(), viewDir.Y(), viewDir.Z()));

            // Up direction
            gp_Dir upDir = vo->UpDirection();
            vp.SetCameraUp(
                ON_3dVector(upDir.X(), upDir.Y(), upDir.Z()));

            // Window size
            const double hw = vo->WindowHorizontalSize();
            const double hh = vo->WindowVerticalSize();

            if (isPerspective) {
                vp.SetProjection(ON::perspective_view);
                // Approximate camera angle from window height
                if (hh > 0.0) {
                    const double halfAngle = std::atan2(hh * 0.5, 1.0);
                    vp.ChangeToPerspectiveProjection(
                        ON_UNSET_VALUE, // target_dist
                        std::tan(halfAngle),
                        true);
                }
            } else {
                vp.SetProjection(ON::parallel_view);
                if (hw > 0.0 && hh > 0.0)
                    vp.SetFrustum(-hw * 0.5, hw * 0.5,
                                  -hh * 0.5, hh * 0.5,
                                   0.1, 1e4);
            }

            model.m_settings.m_named_views.Append(onView);
        }
    }

    // -----------------------------------------------------------------------
    // P4: Text annotations — read XCAF comment notes → ON_Text objects
    // -----------------------------------------------------------------------
    {
        Handle(XCAFDoc_NotesTool) notesTool =
            XCAFDoc_DocumentTool::NotesTool(doc->Main());

        TDF_LabelSequence noteLabels;
        notesTool->GetNotes(noteLabels);

        for (Standard_Integer i = 1; i <= noteLabels.Length(); ++i) {
            const TDF_Label& nl = noteLabels.Value(i);
            Handle(XCAFDoc_NoteComment) nc = XCAFDoc_NoteComment::Get(nl);
            if (nc.IsNull()) continue;

            const TCollection_ExtendedString& comment = nc->Comment();
            ON_wString txt = ExtToONw(comment);
            if (txt.IsEmpty()) continue;

            // Recover placement plane origin from TDataStd_Comment if present
            ON_3dPoint org(0, 0, 0);
            {
                Handle(TDataStd_Comment) coordAttr;
                if (nl.FindAttribute(TDataStd_Comment::GetID(), coordAttr)) {
                    const TCollection_ExtendedString& cs = coordAttr->Get();
                    ON_wString coordW = ExtToONw(cs);
                    ON_String coordS;
                    coordS = coordW;
                    double x = 0, y = 0, z = 0;
                    std::sscanf(coordS.Array(), "%lf %lf %lf", &x, &y, &z);
                    org = ON_3dPoint(x, y, z);
                }
            }

            ON_Plane plane(org, ON_3dVector::ZAxis);
            ON_Text* onText = new ON_Text();
            onText->Create(static_cast<const wchar_t*>(txt), nullptr, plane);

            ON_3dmObjectAttributes* tAttrs = new ON_3dmObjectAttributes();
            {
                Handle(TDataStd_Name) na;
                if (nl.FindAttribute(TDataStd_Name::GetID(), na))
                    tAttrs->m_name = ExtToONw(na->Get());
            }

            model.AddManagedModelGeometryComponent(onText, tAttrs);
            anyOK = true;
        }
    }

    return anyOK || freeShapes.IsEmpty();
}

} // namespace open2open
