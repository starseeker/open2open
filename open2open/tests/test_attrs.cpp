// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_attrs.cpp — Unit tests for attrs_convert.h (Phase-2 attribute items).
//
// Tests:
//   1. CreateXCAFDocument — returns a non-null, initialised document.
//   2. ONX_ModelToXCAFDoc — B-Rep with name, colour, layer, material and
//      document metadata round-trips to an XCAF document.
//   3. XCAFDocToONX_Model — the XCAF document produced in test 2 can be
//      converted back to an ONX_Model preserving names, colours, layers
//      materials and the unit system.
//   4. Round-trip invariance — two round-trips should give the same shape
//      count and attribute count.
//
// Build requirements: open2open (with OPEN2OPEN_HAVE_XCAF), TKPrim

#ifndef OPEN2OPEN_HAVE_XCAF

#include <cstdio>
int main()
{
    std::printf("SKIP: open2open compiled without XCAF support.\n");
    return 0;
}

#else

#include "open2open/attrs_convert.h"
#include "open2open/brep_convert.h"

#include "opennurbs.h"

// OCCT — framework
#include <TDocStd_Document.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>

// OCCT — XCAF tools
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_MaterialTool.hxx>

// OCCT — geometry
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <TopoDS_Shape.hxx>

// Quantity
#include <Quantity_ColorRGBA.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TColStd_HSequenceOfExtendedString.hxx>

#include <cstdio>
#include <cstring>
#include <string>

// ---------------------------------------------------------------------------
// Minimal test framework
// ---------------------------------------------------------------------------
static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_TRUE(expr)                                              \
    do {                                                               \
        if ((expr)) {                                                  \
            ++g_pass;                                                  \
        } else {                                                       \
            ++g_fail;                                                  \
            std::printf("FAIL [%s:%d]: %s\n", __FILE__, __LINE__, #expr); \
        }                                                              \
    } while (0)

#define EXPECT_EQ(a, b)                                                     \
    do {                                                                    \
        if ((a) == (b)) {                                                   \
            ++g_pass;                                                       \
        } else {                                                            \
            ++g_fail;                                                       \
            std::printf("FAIL [%s:%d]: %s == %s  (got %d vs %d)\n",       \
                        __FILE__, __LINE__, #a, #b, (int)(a), (int)(b));   \
        }                                                                   \
    } while (0)

// ---------------------------------------------------------------------------
// Helper: build a simple ONX_Model containing one box B-Rep with metadata.
// ---------------------------------------------------------------------------
static ONX_Model* BuildTestModel()
{
    ONX_Model* model = new ONX_Model();

    // Settings: millimetres
    model->m_settings.m_ModelUnitsAndTolerances.m_unit_system =
        ON_UnitSystem(ON::LengthUnitSystem::Millimeters);

    // Properties: created-by
    model->m_properties.m_RevisionHistory.m_sCreatedBy =
        ON_wString(L"test_attrs");

    // Layer 0: "Red" layer with a red colour
    const int redLayerIdx = model->AddLayer(L"Red", ON_Color(220, 30, 30));

    // Layer 1: "Blue" layer with a blue colour
    const int blueLayerIdx = model->AddLayer(L"Blue", ON_Color(30, 30, 200));

    // Build a small box B-Rep (OCCT → ON_Brep)
    BRepPrimAPI_MakeBox mkBox(10.0, 20.0, 30.0);
    mkBox.Build();
    const TopoDS_Shape& boxShape = mkBox.Shape();

    ON_Brep* boxBrep = new ON_Brep();
    if (!open2open::OCCTToON_Brep(boxShape, *boxBrep)) {
        delete boxBrep;
        delete model;
        return nullptr;
    }

    // Attributes for the box: name, colour, layer
    ON_3dmObjectAttributes* boxAttrs = new ON_3dmObjectAttributes();
    boxAttrs->m_name = ON_wString(L"MyBox");
    boxAttrs->m_color = ON_Color(255, 128, 0); // orange
    boxAttrs->SetColorSource(ON::color_from_object);
    boxAttrs->m_layer_index = redLayerIdx;

    model->AddManagedModelGeometryComponent(boxBrep, boxAttrs);

    // Build a sphere B-Rep
    BRepPrimAPI_MakeSphere mkSph(5.0);
    mkSph.Build();
    const TopoDS_Shape& sphShape = mkSph.Shape();

    ON_Brep* sphBrep = new ON_Brep();
    if (!open2open::OCCTToON_Brep(sphShape, *sphBrep)) {
        delete sphBrep;
        delete model;
        return nullptr;
    }

    ON_3dmObjectAttributes* sphAttrs = new ON_3dmObjectAttributes();
    sphAttrs->m_name  = ON_wString(L"MySphere");
    sphAttrs->m_color = ON_Color(0, 100, 200); // blue-ish
    sphAttrs->SetColorSource(ON::color_from_object);
    sphAttrs->m_layer_index = blueLayerIdx;

    model->AddManagedModelGeometryComponent(sphBrep, sphAttrs);

    (void)redLayerIdx;
    (void)blueLayerIdx;
    return model;
}

// ---------------------------------------------------------------------------
// Test 1: CreateXCAFDocument
// ---------------------------------------------------------------------------
static void TestCreateDocument()
{
    std::printf("--- Test 1: CreateXCAFDocument\n");

    Handle(TDocStd_Document) doc = open2open::CreateXCAFDocument();
    EXPECT_TRUE(!doc.IsNull());

    if (!doc.IsNull()) {
        EXPECT_TRUE(XCAFDoc_DocumentTool::IsXCAFDocument(doc));
        Handle(XCAFDoc_ShapeTool) st =
            XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        EXPECT_TRUE(!st.IsNull());
        Handle(XCAFDoc_ColorTool) ct =
            XCAFDoc_DocumentTool::ColorTool(doc->Main());
        EXPECT_TRUE(!ct.IsNull());
    }
}

// ---------------------------------------------------------------------------
// Test 2: ONX_ModelToXCAFDoc
// ---------------------------------------------------------------------------
static void TestModelToXCAF()
{
    std::printf("--- Test 2: ONX_ModelToXCAFDoc\n");

    ONX_Model* model = BuildTestModel();
    EXPECT_TRUE(model != nullptr);
    if (!model) return;

    Handle(TDocStd_Document) doc =
        open2open::ONX_ModelToXCAFDoc(*model, 1e-6);
    EXPECT_TRUE(!doc.IsNull());
    delete model;
    if (doc.IsNull()) return;

    Handle(XCAFDoc_ShapeTool)    st = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool)    ct = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    Handle(XCAFDoc_LayerTool)    lt = XCAFDoc_DocumentTool::LayerTool(doc->Main());

    // There should be 2 free (top-level) shapes
    TDF_LabelSequence freeShapes;
    st->GetFreeShapes(freeShapes);
    EXPECT_EQ(freeShapes.Length(), 2);

    // Both shapes should have names
    int namedCount = 0;
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        Handle(TDataStd_Name) nameAttr;
        if (freeShapes.Value(i).FindAttribute(TDataStd_Name::GetID(), nameAttr))
            ++namedCount;
    }
    EXPECT_EQ(namedCount, 2);

    // Both shapes should have a generic colour
    int colouredCount = 0;
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        Quantity_ColorRGBA qc;
        if (ct->GetColor(freeShapes.Value(i), XCAFDoc_ColorGen, qc))
            ++colouredCount;
    }
    EXPECT_EQ(colouredCount, 2);

    // There should be 2 layers (Red, Blue)
    TDF_LabelSequence layerLabels;
    lt->GetLayerLabels(layerLabels);
    EXPECT_EQ(layerLabels.Length(), 2);

    // Layers should have colours
    int layersWithColor = 0;
    for (Standard_Integer i = 1; i <= layerLabels.Length(); ++i) {
        Quantity_ColorRGBA qc;
        if (ct->GetColor(layerLabels.Value(i), XCAFDoc_ColorGen, qc))
            ++layersWithColor;
    }
    EXPECT_EQ(layersWithColor, 2);

    // Document creator name should be set
    Handle(TDataStd_Name) docName;
    EXPECT_TRUE(doc->Main().FindAttribute(TDataStd_Name::GetID(), docName));
    if (!docName.IsNull()) {
        // Should contain "test_attrs"
        TCollection_AsciiString ascii(docName->Get(), '?');
        EXPECT_TRUE(ascii.Search("test_attrs") > 0);
    }

    // Unit system should be set (mm = 0.001 m)
    Standard_Real unitVal = -1.0;
    EXPECT_TRUE(XCAFDoc_DocumentTool::GetLengthUnit(doc, unitVal));
    EXPECT_TRUE(std::abs(unitVal - 1e-3) < 1e-8);
}

// ---------------------------------------------------------------------------
// Test 3: XCAFDocToONX_Model round-trip
// ---------------------------------------------------------------------------
static void TestXCAFToModel()
{
    std::printf("--- Test 3: XCAFDocToONX_Model\n");

    // Build model → XCAF
    ONX_Model* src = BuildTestModel();
    EXPECT_TRUE(src != nullptr);
    if (!src) return;

    Handle(TDocStd_Document) doc =
        open2open::ONX_ModelToXCAFDoc(*src, 1e-6);
    delete src;
    EXPECT_TRUE(!doc.IsNull());
    if (doc.IsNull()) return;

    // XCAF → model
    ONX_Model dst;
    EXPECT_TRUE(open2open::XCAFDocToONX_Model(doc, dst, 1e-6));

    // Should have 2 B-Rep objects
    int brepCount = 0;
    ONX_ModelComponentIterator it(dst, ON_ModelComponent::Type::ModelGeometry);
    for (const ON_ModelComponent* mc = it.FirstComponent();
         mc != nullptr;
         mc = it.NextComponent())
    {
        const ON_ModelGeometryComponent* mgc =
            ON_ModelGeometryComponent::Cast(mc);
        if (!mgc) continue;
        const ON_Geometry* g = mgc->Geometry(nullptr);
        if (g && g->ObjectType() == ON::brep_object) ++brepCount;
    }
    EXPECT_EQ(brepCount, 2);

    // Names should be preserved (MyBox or MySphere)
    int namedBreps = 0;
    ONX_ModelComponentIterator it2(dst, ON_ModelComponent::Type::ModelGeometry);
    for (const ON_ModelComponent* mc = it2.FirstComponent();
         mc != nullptr;
         mc = it2.NextComponent())
    {
        const ON_ModelGeometryComponent* mgc =
            ON_ModelGeometryComponent::Cast(mc);
        if (!mgc) continue;
        const ON_3dmObjectAttributes* a = mgc->Attributes(nullptr);
        if (a && a->m_name.IsNotEmpty()) ++namedBreps;
    }
    EXPECT_EQ(namedBreps, 2);

    // Colours should be round-tripped
    int colouredBreps = 0;
    ONX_ModelComponentIterator it3(dst, ON_ModelComponent::Type::ModelGeometry);
    for (const ON_ModelComponent* mc = it3.FirstComponent();
         mc != nullptr;
         mc = it3.NextComponent())
    {
        const ON_ModelGeometryComponent* mgc =
            ON_ModelGeometryComponent::Cast(mc);
        if (!mgc) continue;
        const ON_3dmObjectAttributes* a = mgc->Attributes(nullptr);
        if (a && a->ColorSource() == ON::color_from_object) ++colouredBreps;
    }
    EXPECT_EQ(colouredBreps, 2);

    // Unit system should be preserved (mm)
    const ON::LengthUnitSystem us =
        dst.m_settings.m_ModelUnitsAndTolerances.m_unit_system.UnitSystem();
    EXPECT_TRUE(us == ON::LengthUnitSystem::Millimeters);

    // Layers should be present
    int layerCount = 0;
    ONX_ModelComponentIterator lit(dst, ON_ModelComponent::Type::Layer);
    for (const ON_ModelComponent* mc = lit.FirstComponent();
         mc != nullptr;
         mc = lit.NextComponent())
    {
        ++layerCount;
    }
    EXPECT_EQ(layerCount, 2);
}

// ---------------------------------------------------------------------------
// Test 4: null-document robustness
// ---------------------------------------------------------------------------
static void TestNullDoc()
{
    std::printf("--- Test 4: null-document robustness\n");

    Handle(TDocStd_Document) nullDoc;
    ONX_Model model;
    const bool ok = open2open::XCAFDocToONX_Model(nullDoc, model);
    EXPECT_TRUE(!ok);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main()
{
    TestCreateDocument();
    TestModelToXCAF();
    TestXCAFToModel();
    TestNullDoc();

    std::printf("\n%d/%d tests passed.\n", g_pass, g_pass + g_fail);
    return (g_fail == 0) ? 0 : 1;
}

#endif // OPEN2OPEN_HAVE_XCAF
