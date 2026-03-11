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
#include <XCAFDoc_ViewTool.hxx>
#include <XCAFDoc_NotesTool.hxx>

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
// Test 5: Instance definitions / blocks (P3)
// ---------------------------------------------------------------------------
// Build an ONX_Model with one block definition containing two boxes, then
// place two instances of that block with different transforms.
// Verify that ONX_ModelToXCAFDoc creates an XCAF assembly, and that
// XCAFDocToONX_Model restores the ON_InstanceDefinition + ON_InstanceRef.
// ---------------------------------------------------------------------------
static void TestInstanceRoundTrip()
{
    std::printf("--- Test 5: instance definition round-trip\n");

    // ---- Build source model ----
    ONX_Model src;
    src.m_settings.m_ModelUnitsAndTolerances.m_unit_system =
        ON_UnitSystem(ON::LengthUnitSystem::Millimeters);

    // Add two boxes as block member geometry
    ON_Brep* box1 = new ON_Brep();
    {
        TopoDS_Shape s = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
        bool ok = open2open::OCCTToON_Brep(s, *box1, 1e-3);
        EXPECT_TRUE(ok);
        (void)ok;
    }
    ON_Brep* box2 = new ON_Brep();
    {
        TopoDS_Shape s = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape();
        bool ok = open2open::OCCTToON_Brep(s, *box2, 1e-3);
        EXPECT_TRUE(ok);
        (void)ok;
    }

    ON_3dmObjectAttributes* a1 = new ON_3dmObjectAttributes();
    a1->m_name = L"Box1";
    ON_3dmObjectAttributes* a2 = new ON_3dmObjectAttributes();
    a2->m_name = L"Box2";

    auto ref1 = src.AddManagedModelGeometryComponent(box1, a1);
    auto ref2 = src.AddManagedModelGeometryComponent(box2, a2);
    ON_UUID uuid1 = ref1.ModelComponent()->Id();
    ON_UUID uuid2 = ref2.ModelComponent()->Id();

    // Create an instance definition containing both boxes
    ON_InstanceDefinition* idef = new ON_InstanceDefinition();
    idef->SetName(L"DualBox");
    ON_SimpleArray<ON_UUID> members;
    members.Append(uuid1);
    members.Append(uuid2);
    idef->SetInstanceGeometryIdList(members);
    auto idefRef = src.AddManagedModelComponent(idef, false);
    ON_UUID idefUuid = idefRef.ModelComponent()->Id();

    // Place the block twice: once at origin, once translated by (50,0,0)
    ON_InstanceRef* iref_a = new ON_InstanceRef();
    iref_a->m_instance_definition_uuid = idefUuid;
    iref_a->m_xform = ON_Xform::IdentityTransformation;
    ON_3dmObjectAttributes* ia_attrs = new ON_3dmObjectAttributes();
    ia_attrs->m_name = L"Instance_A";
    src.AddManagedModelGeometryComponent(iref_a, ia_attrs);

    ON_InstanceRef* iref_b = new ON_InstanceRef();
    iref_b->m_instance_definition_uuid = idefUuid;
    iref_b->m_xform = ON_Xform::TranslationTransformation(
        ON_3dVector(50.0, 0.0, 0.0));
    ON_3dmObjectAttributes* ib_attrs = new ON_3dmObjectAttributes();
    ib_attrs->m_name = L"Instance_B";
    src.AddManagedModelGeometryComponent(iref_b, ib_attrs);

    // ---- Convert ONX → XCAF ----
    auto doc = open2open::ONX_ModelToXCAFDoc(src, 1e-3);
    EXPECT_TRUE(!doc.IsNull());
    if (doc.IsNull()) return;

    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(doc->Main());

    // Verify an assembly exists in the XCAF document
    TDF_LabelSequence freeShapes;
    shapeTool->GetFreeShapes(freeShapes);

    bool hasAssembly = false;
    for (Standard_Integer i = 1; i <= freeShapes.Length(); ++i) {
        if (XCAFDoc_ShapeTool::IsAssembly(freeShapes.Value(i))) {
            hasAssembly = true;
            break;
        }
    }
    EXPECT_TRUE(hasAssembly);

    // ---- Convert XCAF → ONX_Model ----
    ONX_Model dst;
    bool ok2 = open2open::XCAFDocToONX_Model(doc, dst, 1e-3);
    EXPECT_TRUE(ok2);

    // Count instance definitions and instance refs in restored model
    int idefCount = 0;
    {
        ONX_ModelComponentIterator it(dst,
            ON_ModelComponent::Type::InstanceDefinition);
        for (const ON_ModelComponent* mc = it.FirstComponent();
             mc != nullptr;
             mc = it.NextComponent())
            ++idefCount;
    }
    EXPECT_EQ(idefCount, 1); // one block definition

    int irefCount = 0;
    {
        ONX_ModelComponentIterator it(dst,
            ON_ModelComponent::Type::ModelGeometry);
        for (const ON_ModelComponent* mc = it.FirstComponent();
             mc != nullptr;
             mc = it.NextComponent())
        {
            const ON_ModelGeometryComponent* mgc =
                ON_ModelGeometryComponent::Cast(mc);
            if (!mgc) continue;
            const ON_Geometry* g = mgc->Geometry(nullptr);
            if (g && g->ObjectType() == ON::instance_reference)
                ++irefCount;
        }
    }
    EXPECT_EQ(irefCount, 2); // two instances
}

// ---------------------------------------------------------------------------
// Test 6: Named views round-trip (P3)
// ---------------------------------------------------------------------------
static void TestNamedViewRoundTrip()
{
    std::printf("--- Test 6: named view round-trip\n");

    ONX_Model src;
    src.m_settings.m_ModelUnitsAndTolerances.m_unit_system =
        ON_UnitSystem(ON::LengthUnitSystem::Millimeters);

    // Add a perspective view
    {
        ON_3dmView v;
        v.m_name = L"Perspective";
        v.m_vp.SetProjection(ON::perspective_view);
        v.m_vp.SetCameraLocation(ON_3dPoint(100, 200, 300));
        v.m_vp.SetCameraDirection(ON_3dVector(-1, -2, -3));
        v.m_vp.SetCameraUp(ON_3dVector(0, 0, 1));
        v.m_vp.SetFrustum(-5, 5, -4, 4, 1.0, 1000.0);
        src.m_settings.m_named_views.Append(v);
    }
    // Add a parallel view
    {
        ON_3dmView v;
        v.m_name = L"Top";
        v.m_vp.SetProjection(ON::parallel_view);
        v.m_vp.SetCameraLocation(ON_3dPoint(0, 0, 500));
        v.m_vp.SetCameraDirection(ON_3dVector(0, 0, -1));
        v.m_vp.SetCameraUp(ON_3dVector(0, 1, 0));
        v.m_vp.SetFrustum(-50, 50, -50, 50, 0.1, 1e4);
        src.m_settings.m_named_views.Append(v);
    }

    auto doc = open2open::ONX_ModelToXCAFDoc(src, 1e-3);
    EXPECT_TRUE(!doc.IsNull());
    if (doc.IsNull()) return;

    // Check XCAF view count
    Handle(XCAFDoc_ViewTool) viewTool =
        XCAFDoc_DocumentTool::ViewTool(doc->Main());
    TDF_LabelSequence viewLabels;
    viewTool->GetViewLabels(viewLabels);
    EXPECT_EQ(viewLabels.Length(), 2);

    // Round-trip back to ONX_Model
    ONX_Model dst;
    bool ok = open2open::XCAFDocToONX_Model(doc, dst, 1e-3);
    EXPECT_TRUE(ok);

    // Named views should be restored
    EXPECT_EQ(dst.m_settings.m_named_views.Count(), 2);

    if (dst.m_settings.m_named_views.Count() >= 1) {
        const ON_3dmView& v0 = dst.m_settings.m_named_views[0];
        const ON_wString& n0 = v0.m_name;
        EXPECT_TRUE(n0 == L"Perspective");
    }
    if (dst.m_settings.m_named_views.Count() >= 2) {
        const ON_3dmView& v1 = dst.m_settings.m_named_views[1];
        EXPECT_TRUE(v1.m_name == L"Top");
        EXPECT_TRUE(!v1.m_vp.IsPerspectiveProjection());
    }
}

// ---------------------------------------------------------------------------
// Test 7: Layer hierarchy (P3) — parent/child ON_Layer ↔ XCAF "Parent/Child"
// ---------------------------------------------------------------------------
static void TestLayerHierarchyRoundTrip()
{
    std::printf("--- Test 7: layer hierarchy round-trip\n");

    ONX_Model src;
    src.m_settings.m_ModelUnitsAndTolerances.m_unit_system =
        ON_UnitSystem(ON::LengthUnitSystem::Millimeters);

    // Add parent layer
    const int parentIdx = src.AddLayer(L"Mechanical", ON_Color::UnsetColor);

    // Get parent layer UUID
    ON_ModelComponentReference pref =
        src.ComponentFromIndex(ON_ModelComponent::Type::Layer, parentIdx);
    const ON_Layer* pLayer = ON_Layer::Cast(pref.ModelComponent());
    EXPECT_TRUE(pLayer != nullptr);
    if (!pLayer) return;
    const ON_UUID parentUuid = pLayer->Id();

    // Add child layer with parent set
    ON_Layer* childLayer = new ON_Layer();
    childLayer->SetName(L"Bolts");
    childLayer->SetParentLayerId(parentUuid);
    childLayer->m_color = ON_Color(255, 0, 0);
    const int childIdx = src.AddManagedModelComponent(
        childLayer, false).ModelComponentIndex();
    EXPECT_TRUE(childIdx >= 0);

    // Add a box on the child layer
    {
        TopoDS_Shape s = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape();
        ON_Brep* brep = new ON_Brep();
        open2open::OCCTToON_Brep(s, *brep, 1e-3);
        ON_3dmObjectAttributes* a = new ON_3dmObjectAttributes();
        a->m_name = L"Bolt";
        a->m_layer_index = childIdx;
        src.AddManagedModelGeometryComponent(brep, a);
    }

    // Convert to XCAF
    auto doc = open2open::ONX_ModelToXCAFDoc(src, 1e-3);
    EXPECT_TRUE(!doc.IsNull());
    if (doc.IsNull()) return;

    // Verify XCAF layer names include "Mechanical/Bolts"
    Handle(XCAFDoc_LayerTool) layerTool2 =
        XCAFDoc_DocumentTool::LayerTool(doc->Main());
    TDF_LabelSequence layerLabels;
    layerTool2->GetLayerLabels(layerLabels);
    bool foundChild = false;
    for (Standard_Integer i = 1; i <= layerLabels.Length(); ++i) {
        TCollection_ExtendedString ln;
        layerTool2->GetLayer(layerLabels.Value(i), ln);
        // Check if name contains "Mechanical/Bolts"
        TCollection_ExtendedString expected("Mechanical/Bolts");
        if (ln == expected) { foundChild = true; break; }
    }
    EXPECT_TRUE(foundChild);

    // Round-trip back
    ONX_Model dst;
    bool ok = open2open::XCAFDocToONX_Model(doc, dst, 1e-3);
    EXPECT_TRUE(ok);

    // Find child layer in dst and verify it has a parent
    bool childHasParent = false;
    {
        ONX_ModelComponentIterator lit(dst, ON_ModelComponent::Type::Layer);
        for (const ON_ModelComponent* mc = lit.FirstComponent();
             mc != nullptr; mc = lit.NextComponent())
        {
            const ON_Layer* l = ON_Layer::Cast(mc);
            if (!l) continue;
            if (l->Name() == ON_wString(L"Bolts") &&
                l->ParentLayerId() != ON_nil_uuid)
            {
                childHasParent = true;
                break;
            }
        }
    }
    EXPECT_TRUE(childHasParent);
}

// ---------------------------------------------------------------------------
// Test 8: Point cloud round-trip (P4)
// ---------------------------------------------------------------------------
static void TestPointCloudRoundTrip()
{
    std::printf("--- Test 8: point cloud round-trip\n");

    ONX_Model src;
    src.m_settings.m_ModelUnitsAndTolerances.m_unit_system =
        ON_UnitSystem(ON::LengthUnitSystem::Millimeters);

    // Create a 5-point cloud
    ON_PointCloud* pc = new ON_PointCloud();
    pc->m_P.Append(ON_3dPoint(0, 0, 0));
    pc->m_P.Append(ON_3dPoint(1, 0, 0));
    pc->m_P.Append(ON_3dPoint(0, 1, 0));
    pc->m_P.Append(ON_3dPoint(0, 0, 1));
    pc->m_P.Append(ON_3dPoint(1, 1, 1));
    ON_3dmObjectAttributes* a = new ON_3dmObjectAttributes();
    a->m_name = L"TestCloud";
    src.AddManagedModelGeometryComponent(pc, a);

    auto doc = open2open::ONX_ModelToXCAFDoc(src, 1e-3);
    EXPECT_TRUE(!doc.IsNull());
    if (doc.IsNull()) return;

    // Round-trip back
    ONX_Model dst;
    bool ok = open2open::XCAFDocToONX_Model(doc, dst, 1e-3);
    EXPECT_TRUE(ok);

    // Find point cloud in dst
    bool foundCloud = false;
    {
        ONX_ModelComponentIterator it(dst,
            ON_ModelComponent::Type::ModelGeometry);
        for (const ON_ModelComponent* mc = it.FirstComponent();
             mc != nullptr; mc = it.NextComponent())
        {
            const ON_ModelGeometryComponent* mgc =
                ON_ModelGeometryComponent::Cast(mc);
            if (!mgc) continue;
            const ON_Geometry* g = mgc->Geometry(nullptr);
            if (!g) continue;
            const ON_PointCloud* rpc = ON_PointCloud::Cast(g);
            if (rpc && rpc->PointCount() == 5) {
                foundCloud = true;
                break;
            }
        }
    }
    EXPECT_TRUE(foundCloud);
}

// ---------------------------------------------------------------------------
// Test 9: Text annotation round-trip (P4)
// ---------------------------------------------------------------------------
static void TestTextAnnotationRoundTrip()
{
    std::printf("--- Test 9: text annotation round-trip\n");

    ONX_Model src;

    // Add text annotation
    ON_Text* txt = new ON_Text();
    ON_Plane plane(ON_3dPoint(5, 10, 0), ON_3dVector::ZAxis);
    txt->Create(L"Hello World", nullptr, plane);
    ON_3dmObjectAttributes* a = new ON_3dmObjectAttributes();
    a->m_name = L"Label1";
    src.AddManagedModelGeometryComponent(txt, a);

    auto doc = open2open::ONX_ModelToXCAFDoc(src, 1e-3);
    EXPECT_TRUE(!doc.IsNull());
    if (doc.IsNull()) return;

    // Check XCAF has a note
    Handle(XCAFDoc_NotesTool) notesTool =
        XCAFDoc_DocumentTool::NotesTool(doc->Main());
    TDF_LabelSequence noteLabels;
    notesTool->GetNotes(noteLabels);
    EXPECT_EQ(noteLabels.Length(), 1);

    // Round-trip back
    ONX_Model dst;
    bool ok = open2open::XCAFDocToONX_Model(doc, dst, 1e-3);
    EXPECT_TRUE(ok);

    // Find text in dst
    bool foundText = false;
    {
        ONX_ModelComponentIterator it(dst,
            ON_ModelComponent::Type::ModelGeometry);
        for (const ON_ModelComponent* mc = it.FirstComponent();
             mc != nullptr; mc = it.NextComponent())
        {
            const ON_ModelGeometryComponent* mgc =
                ON_ModelGeometryComponent::Cast(mc);
            if (!mgc) continue;
            const ON_Geometry* g = mgc->Geometry(nullptr);
            if (!g || g->ObjectType() != ON::annotation_object) continue;
            const ON_Annotation* ann = ON_Annotation::Cast(g);
            if (ann) {
                const ON_wString t = ann->PlainText();
                if (t == ON_wString(L"Hello World")) { foundText = true; break; }
            }
        }
    }
    EXPECT_TRUE(foundText);
}

// ---------------------------------------------------------------------------
// Test 10: Embedded texture map round-trip (P4)
// ---------------------------------------------------------------------------
static void TestTextureMapRoundTrip()
{
    std::printf("--- Test 10: texture map round-trip\n");

    ONX_Model src;
    src.m_settings.m_ModelUnitsAndTolerances.m_unit_system =
        ON_UnitSystem(ON::LengthUnitSystem::Millimeters);

    // Add material with a texture reference
    ON_Material* mat = new ON_Material();
    mat->SetName(L"Painted");
    mat->SetDiffuse(ON_Color(200, 100, 50));
    {
        ON_Texture tex;
        tex.m_type = ON_Texture::TYPE::bitmap_texture;
        tex.m_image_file_reference =
            ON_FileReference::CreateFromFullPath(
                L"/textures/wood.png", false, false);
        mat->AddTexture(tex);
    }
    const int matIdx = src.AddManagedModelComponent(
        mat, false).ModelComponentIndex();

    // Add a box using this material
    {
        TopoDS_Shape s = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape();
        ON_Brep* brep = new ON_Brep();
        open2open::OCCTToON_Brep(s, *brep, 1e-3);
        ON_3dmObjectAttributes* a = new ON_3dmObjectAttributes();
        a->m_name = L"Box";
        a->m_material_index = matIdx;
        src.AddManagedModelGeometryComponent(brep, a);
    }

    auto doc = open2open::ONX_ModelToXCAFDoc(src, 1e-3);
    EXPECT_TRUE(!doc.IsNull());
    if (doc.IsNull()) return;

    // Round-trip
    ONX_Model dst;
    bool ok = open2open::XCAFDocToONX_Model(doc, dst, 1e-3);
    EXPECT_TRUE(ok);

    // Check texture was restored
    bool foundTexture = false;
    {
        ONX_ModelComponentIterator mit(dst,
            ON_ModelComponent::Type::RenderMaterial);
        for (const ON_ModelComponent* mc = mit.FirstComponent();
             mc != nullptr; mc = mit.NextComponent())
        {
            const ON_Material* m = ON_Material::Cast(mc);
            if (!m) continue;
            if (m->m_textures.Count() >= 1) {
                const ON_wString& fp =
                    m->m_textures[0].m_image_file_reference.FullPath();
                if (fp == ON_wString(L"/textures/wood.png")) {
                    foundTexture = true;
                    break;
                }
            }
        }
    }
    EXPECT_TRUE(foundTexture);
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
    TestInstanceRoundTrip();
    TestNamedViewRoundTrip();
    TestLayerHierarchyRoundTrip();
    TestPointCloudRoundTrip();
    TestTextAnnotationRoundTrip();
    TestTextureMapRoundTrip();

    std::printf("\n%d/%d tests passed.\n", g_pass, g_pass + g_fail);
    return (g_fail == 0) ? 0 : 1;
}

#endif // OPEN2OPEN_HAVE_XCAF
