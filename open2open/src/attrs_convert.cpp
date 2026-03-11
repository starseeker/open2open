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

// OCCT — colour
#include <Quantity_Color.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TCollection_HAsciiString.hxx>

// OCCT — shape
#include <TopoDS_Shape.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>

// OCCT — transforms (for instance placement)
#include <gp_Trsf.hxx>
#include <TopLoc_Location.hxx>
#include <XCAFDoc_Location.hxx>

#include <map>
#include <set>
#include <string>
#include <vector>
#include <array>
#include <functional>

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
        ONX_ModelComponentIterator lit(model, ON_ModelComponent::Type::Layer);
        for (const ON_ModelComponent* mc = lit.FirstComponent();
             mc != nullptr;
             mc = lit.NextComponent())
        {
            const ON_Layer* layer = ON_Layer::Cast(mc);
            if (!layer) continue;

            const int idx = layer->Index();
            const ON_wString& name = layer->Name();

            TCollection_ExtendedString extName =
                name.IsNotEmpty() ? ONwToExt(name)
                                  : TCollection_ExtendedString("Layer");

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

        // Skip instance references — handled in the P3 pass below
        if (geom->ObjectType() == ON::instance_reference)
            continue;

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
        } else {
            // Mesh and other types: skip for now
            continue;
        }
        if (shape.IsNull()) continue;

        // Add to XCAF shape tool
        TDF_Label label = shapeTool->AddShape(shape);
        if (label.IsNull()) continue;

        // Record UUID → label for instance-definition assembly pass
        uuidToLabel[mc->Id()] = label;

        const ON_3dmObjectAttributes* attrs = mgc->Attributes(nullptr);

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
    // P1: Layer table — each XCAF layer label → ON_Layer
    // Map: layer label tag → ON layer index
    // -----------------------------------------------------------------------
    std::map<Standard_Integer, int> layerTagToIndex;
    {
        TDF_LabelSequence layerLabels;
        layerTool->GetLayerLabels(layerLabels);
        for (Standard_Integer i = 1; i <= layerLabels.Length(); ++i) {
            const TDF_Label& ll = layerLabels.Value(i);

            TCollection_ExtendedString layerName;
            layerTool->GetLayer(ll, layerName);

            ON_Layer onLayer;
            if (!layerName.IsEmpty())
                onLayer.SetName(ExtToONw(layerName));
            else
                onLayer.SetName(L"Layer");

            // Layer colour stored as generic colour on the layer label
            Quantity_ColorRGBA qc;
            if (colorTool->GetColor(ll, XCAFDoc_ColorGen, qc)) {
                onLayer.m_color = RGBAToONColor(qc);
            }

            const int layerIdx = model.AddLayer(
                static_cast<const wchar_t*>(onLayer.Name()),
                onLayer.m_color);

            layerTagToIndex[ll.Tag()] = layerIdx;
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
            for (Standard_Integer k = 1; k <= comps.Length(); ++k)
                collectAndAddSimple(comps.Value(k));
            return;
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

    return anyOK || freeShapes.IsEmpty();
}

} // namespace open2open
