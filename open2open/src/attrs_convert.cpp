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

#include <map>
#include <string>
#include <vector>

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
    // P1/P2: Geometry objects
    // -----------------------------------------------------------------------
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
    // P1/P2: Top-level shapes
    // -----------------------------------------------------------------------
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);

    bool anyOK = false;
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        const TDF_Label& label = freeShapes.Value(i);
        const TopoDS_Shape shape = shapeTool->GetShape(label);
        if (shape.IsNull()) continue;

        ON_Brep* brep = new ON_Brep();
        if (!OCCTToON_Brep(shape, *brep, tol)) {
            delete brep;
            continue;
        }
        if (!brep->IsValid() || brep->m_F.Count() == 0) {
            delete brep;
            continue;
        }

        // Build per-object attributes
        ON_3dmObjectAttributes* attrs = new ON_3dmObjectAttributes();

        // P1: Name
        {
            Handle(TDataStd_Name) nameAttr;
            if (label.FindAttribute(TDataStd_Name::GetID(), nameAttr)) {
                attrs->m_name = ExtToONw(nameAttr->Get());
            }
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
                const TCollection_ExtendedString& lname =
                    layerNames->Value(1);
                // Find the corresponding ON layer index
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

        model.AddManagedModelGeometryComponent(brep, attrs);
        anyOK = true;
    }

    return anyOK || freeShapes.IsEmpty();
}

} // namespace open2open
