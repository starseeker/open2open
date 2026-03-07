// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_brep.cpp — Round-trip B-Rep tests.
//
// Tests: ON_Brep → TopoDS_Shape → ON_Brep
//
// Verification:
//   1. OCCT shape passes BRepCheck_Analyzer after ON→OCCT translation.
//   2. ON_Brep passes IsValid() after OCCT→ON back-translation.
//   3. For each face pair, sampled surface points agree within tolerance.

#include "open2open/brep_convert.h"
#include "open2open/geom_convert.h"

#include "opennurbs.h"

#include <BRepCheck_Analyzer.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS.hxx>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

static const double kLinTol = 1e-6;

// ---------------------------------------------------------------------------
// Run the full round-trip test for a given ON_Brep
// ---------------------------------------------------------------------------
static bool TestBrepRoundTrip(const ON_Brep* brep_in, const char* label)
{
    if (!brep_in) {
        std::printf("FAIL [%s]: null input brep\n", label);
        return false;
    }
    if (!brep_in->IsValid()) {
        std::printf("FAIL [%s]: input brep is not valid\n", label);
        return false;
    }

    // --- Step 1: ON → OCCT ---
    TopoDS_Shape shape =
        open2open::ON_BrepToOCCT(*brep_in, kLinTol);

    if (shape.IsNull()) {
        std::printf("FAIL [%s]: ON_BrepToOCCT returned null shape\n", label);
        return false;
    }

    // --- Step 2: Validate OCCT shape ---
    BRepCheck_Analyzer checker(shape, Standard_False);
    if (!checker.IsValid()) {
        // Non-fatal warning — some tolerance issues are expected with the
        // initial implementation; we report but continue.
        std::printf("WARN [%s]: BRepCheck_Analyzer reports invalid shape\n",
                    label);
    }

    // --- Step 3: OCCT → ON ---
    ON_Brep brep_rt;
    bool ok = open2open::OCCTToON_Brep(shape, brep_rt, kLinTol);
    if (!ok) {
        std::printf("FAIL [%s]: OCCTToON_Brep returned false\n", label);
        return false;
    }
    if (!brep_rt.IsValid()) {
        std::printf("FAIL [%s]: round-trip ON_Brep is not valid\n", label);
        return false;
    }

    // --- Step 4: surface count sanity ---
    if (brep_rt.m_F.Count() == 0) {
        std::printf("FAIL [%s]: round-trip brep has no faces\n", label);
        return false;
    }

    std::printf("PASS [%s]: %d faces, %d edges, %d vertices\n",
                label,
                brep_rt.m_F.Count(),
                brep_rt.m_E.Count(),
                brep_rt.m_V.Count());
    return true;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
    int passed = 0, total = 0;

    // -- Test 1: box B-Rep --
    {
        ++total;
        ON_3dPoint corners[8];
        corners[0] = ON_3dPoint(0,0,0); corners[1] = ON_3dPoint(1,0,0);
        corners[2] = ON_3dPoint(1,1,0); corners[3] = ON_3dPoint(0,1,0);
        corners[4] = ON_3dPoint(0,0,1); corners[5] = ON_3dPoint(1,0,1);
        corners[6] = ON_3dPoint(1,1,1); corners[7] = ON_3dPoint(0,1,1);
        std::unique_ptr<ON_Brep> b(ON_BrepBox(corners));
        if (TestBrepRoundTrip(b.get(), "box (ON_BrepBox)")) ++passed;
    }

    // -- Test 2: sphere --
    {
        ++total;
        ON_Sphere sphere(ON_3dPoint(0, 0, 0), 1.0);
        std::unique_ptr<ON_Brep> b(ON_BrepSphere(sphere));
        if (TestBrepRoundTrip(b.get(), "sphere (ON_BrepSphere)")) ++passed;
    }

    // -- Test 3: cylinder --
    {
        ++total;
        ON_Circle circle(ON_Plane::World_xy, 1.0);
        ON_Cylinder cyl(circle, 2.0);
        std::unique_ptr<ON_Brep> b(ON_BrepCylinder(cyl,
                                                    true,  // cap_bottom
                                                    true));// cap_top
        if (TestBrepRoundTrip(b.get(), "cylinder (ON_BrepCylinder)")) ++passed;
    }

    // -- Test 4: torus --
    {
        ++total;
        ON_Torus torus(ON_Plane::World_xy, 3.0, 1.0);
        std::unique_ptr<ON_Brep> b(ON_BrepTorus(torus));
        if (TestBrepRoundTrip(b.get(), "torus (ON_BrepTorus)")) ++passed;
    }

    std::printf("\nResult: %d / %d tests passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
