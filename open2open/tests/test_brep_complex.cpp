// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_brep_complex.cpp — Complex B-Rep stress tests.
//
// Exercises round-trip translation for shapes with:
//   - Apex poles (cone)
//   - Triangular / non-rectangular faces (wedge)
//   - Revolved surfaces and trimmed planes
//   - OCCT-native primitives fed directly into OCCTToON_Brep (no
//     ON→OCCT step) including Geom_Plane, Geom_ConicalSurface,
//     Geom_CylindricalSurface and Geom_SphericalSurface
//   - Surface-point accuracy verification on each face

#include "open2open/brep_convert.h"
#include "open2open/geom_convert.h"

#include "opennurbs.h"

// OCCT topology
#include <BRepCheck_Analyzer.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

// OCCT geometry
#include <Geom_BSplineSurface.hxx>
#include <Geom_Surface.hxx>
#include <GeomConvert.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <gp_Pnt.hxx>

// OCCT primitives (needed for native-OCCT round-trip tests)
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

static const double kLinTol    = 1e-6;
static const double kGeomTol   = 1e-4; // looser for analytical→NURBS approx

// ---------------------------------------------------------------------------
// Utility: sample a point on an ON_BrepFace at (u,v) and return it.
// Returns false if evaluation fails.
// ---------------------------------------------------------------------------
static bool FacePoint(const ON_BrepFace& f, double u, double v, ON_3dPoint& pt)
{
    const ON_Surface* srf = f.SurfaceOf();
    if (!srf) return false;
    return srf->EvPoint(u, v, pt);
}

// ---------------------------------------------------------------------------
// Verify geometric accuracy: for each face in the round-trip brep, sample a
// 5×5 grid of UV points and compare against the corresponding face of the
// OCCT shape (by re-evaluating through the OCCT surface).
// Returns the maximum point error found.
// ---------------------------------------------------------------------------
static double VerifySurfaceAccuracy(const TopoDS_Shape& occt_shape,
                                    const ON_Brep& on_brep)
{
    double max_err = 0.0;

    // Collect OCCT faces in order
    std::vector<TopoDS_Face> occt_faces;
    for (TopExp_Explorer fex(occt_shape, TopAbs_FACE); fex.More(); fex.Next())
        occt_faces.push_back(TopoDS::Face(fex.Current()));

    if (on_brep.m_F.Count() != (int)occt_faces.size())
        return -1.0; // face count mismatch

    for (int fi = 0; fi < on_brep.m_F.Count(); ++fi) {
        const ON_BrepFace& of = on_brep.m_F[fi];
        const TopoDS_Face& tf = occt_faces[fi];

        // Get OCCT surface as BSpline (with UV-bounded fallback)
        Handle(Geom_Surface) gs = BRep_Tool::Surface(tf);
        if (gs.IsNull()) continue;
        Handle(Geom_BSplineSurface) bs =
            Handle(Geom_BSplineSurface)::DownCast(gs);
        if (bs.IsNull()) {
            try {
                bs = GeomConvert::SurfaceToBSplineSurface(gs);
            } catch (...) {}
        }
        if (bs.IsNull()) {
            try {
                double u1, u2, v1, v2;
                BRepTools::UVBounds(tf, u1, u2, v1, v2);
                Handle(Geom_RectangularTrimmedSurface) ts =
                    new Geom_RectangularTrimmedSurface(gs, u1, u2, v1, v2);
                bs = GeomConvert::SurfaceToBSplineSurface(ts);
            } catch (...) {}
        }
        if (bs.IsNull()) continue;

        // Sample ON face UV domain
        const ON_Surface* on_srf = of.SurfaceOf();
        if (!on_srf) continue;
        ON_Interval udom = on_srf->Domain(0);
        ON_Interval vdom = on_srf->Domain(1);
        double ou1, ou2, ov1, ov2;
        bs->Bounds(ou1, ou2, ov1, ov2);

        const int N = 5;
        for (int i = 0; i < N; ++i) {
            double on_u = udom.Min() + udom.Length() * i / (N - 1);
            double oc_u = ou1 + (ou2 - ou1) * i / (N - 1);
            for (int j = 0; j < N; ++j) {
                double on_v = vdom.Min() + vdom.Length() * j / (N - 1);
                double oc_v = ov1 + (ov2 - ov1) * j / (N - 1);
                ON_3dPoint on_pt;
                if (!FacePoint(of, on_u, on_v, on_pt)) continue;
                gp_Pnt oc_pt;
                bs->D0(oc_u, oc_v, oc_pt);
                double dx = on_pt.x - oc_pt.X();
                double dy = on_pt.y - oc_pt.Y();
                double dz = on_pt.z - oc_pt.Z();
                double d = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (d > max_err) max_err = d;
            }
        }
    }
    return max_err;
}

// ---------------------------------------------------------------------------
// Core round-trip test for an ON_Brep input
// ---------------------------------------------------------------------------
static bool TestBrepRT(const ON_Brep* brep_in, const char* label,
                       bool check_geom = true)
{
    if (!brep_in || !brep_in->IsValid()) {
        std::printf("FAIL [%s]: null or invalid input brep\n", label);
        return false;
    }

    // Step 1: ON → OCCT
    TopoDS_Shape shape = open2open::ON_BrepToOCCT(*brep_in, kLinTol);
    if (shape.IsNull()) {
        std::printf("FAIL [%s]: ON_BrepToOCCT returned null shape\n", label);
        return false;
    }

    // Step 2: validate OCCT (non-fatal)
    BRepCheck_Analyzer checker(shape, Standard_False);
    if (!checker.IsValid())
        std::printf("WARN [%s]: BRepCheck_Analyzer reports invalid shape\n", label);

    // Step 3: OCCT → ON
    ON_Brep brep_rt;
    if (!open2open::OCCTToON_Brep(shape, brep_rt, kLinTol)) {
        std::printf("FAIL [%s]: OCCTToON_Brep returned false\n", label);
        return false;
    }
    if (!brep_rt.IsValid()) {
        ON_TextLog log;
        brep_rt.IsValid(&log);
        std::printf("FAIL [%s]: round-trip ON_Brep is not valid\n", label);
        return false;
    }
    if (brep_rt.m_F.Count() != brep_in->m_F.Count()) {
        std::printf("FAIL [%s]: face count mismatch: got %d, expected %d\n",
                    label, brep_rt.m_F.Count(), brep_in->m_F.Count());
        return false;
    }
    if (brep_rt.m_E.Count() != brep_in->m_E.Count()) {
        std::printf("FAIL [%s]: edge count mismatch: got %d, expected %d\n",
                    label, brep_rt.m_E.Count(), brep_in->m_E.Count());
        return false;
    }

    // Step 4: geometric accuracy check
    if (check_geom) {
        double err = VerifySurfaceAccuracy(shape, brep_rt);
        if (err < 0.0) {
            std::printf("WARN [%s]: face count mismatch in geometry check\n", label);
        } else if (err > kGeomTol) {
            std::printf("FAIL [%s]: surface accuracy error = %.3e > %.3e\n",
                        label, err, kGeomTol);
            return false;
        }
    }

    std::printf("PASS [%s]: %d faces, %d edges, %d vertices\n",
                label, brep_rt.m_F.Count(), brep_rt.m_E.Count(),
                brep_rt.m_V.Count());
    return true;
}

// ---------------------------------------------------------------------------
// Core test for an OCCT-native shape (no ON→OCCT step)
// ---------------------------------------------------------------------------
static bool TestOCCTNative(const TopoDS_Shape& shape, const char* label,
                            int expected_faces, int expected_edges)
{
    if (shape.IsNull()) {
        std::printf("FAIL [%s]: null input shape\n", label);
        return false;
    }

    BRepCheck_Analyzer checker(shape, Standard_False);
    if (!checker.IsValid())
        std::printf("WARN [%s]: input shape invalid per BRepCheck_Analyzer\n", label);

    ON_Brep brep_rt;
    if (!open2open::OCCTToON_Brep(shape, brep_rt, kLinTol)) {
        std::printf("FAIL [%s]: OCCTToON_Brep returned false\n", label);
        return false;
    }
    if (!brep_rt.IsValid()) {
        ON_TextLog log;
        brep_rt.IsValid(&log);
        std::printf("FAIL [%s]: resulting ON_Brep is not valid\n", label);
        return false;
    }
    if (brep_rt.m_F.Count() != expected_faces) {
        std::printf("FAIL [%s]: face count %d != expected %d\n",
                    label, brep_rt.m_F.Count(), expected_faces);
        return false;
    }
    if (brep_rt.m_E.Count() != expected_edges) {
        std::printf("FAIL [%s]: edge count %d != expected %d\n",
                    label, brep_rt.m_E.Count(), expected_edges);
        return false;
    }
    std::printf("PASS [%s]: %d faces, %d edges, %d vertices\n",
                label, brep_rt.m_F.Count(), brep_rt.m_E.Count(),
                brep_rt.m_V.Count());
    return true;
}

// ===========================================================================
// Main
// ===========================================================================
int main()
{
    int passed = 0, total = 0;

    // -----------------------------------------------------------------------
    // Group A: ON_Brep → OCCT → ON_Brep round-trip for more complex shapes
    // -----------------------------------------------------------------------

    // A1: cone with bottom cap (apex at top = degenerate vertex/edge)
    {
        ++total;
        ON_Cone cone(ON_Plane::World_xy, 2.0, 1.0);
        std::unique_ptr<ON_Brep> b(ON_BrepCone(cone, true));
        if (TestBrepRT(b.get(), "cone with cap")) ++passed;
    }

    // A2: cone without cap (pure lateral surface, singular apex trim)
    {
        ++total;
        ON_Cone cone(ON_Plane::World_xy, 2.0, 1.0);
        std::unique_ptr<ON_Brep> b(ON_BrepCone(cone, false));
        if (TestBrepRT(b.get(), "cone without cap")) ++passed;
    }

    // A3: wedge — 5 faces (2 triangular + 3 rectangular),
    //     9 edges, 6 vertices
    {
        ++total;
        ON_3dPoint c[6];
        c[0] = ON_3dPoint(0.0, 0.0, 0.0);
        c[1] = ON_3dPoint(1.0, 0.0, 0.0);
        c[2] = ON_3dPoint(0.5, 1.0, 0.0);
        c[3] = ON_3dPoint(0.0, 0.0, 2.0);
        c[4] = ON_3dPoint(1.0, 0.0, 2.0);
        c[5] = ON_3dPoint(0.5, 1.0, 2.0);
        std::unique_ptr<ON_Brep> b(ON_BrepWedge(c));
        if (TestBrepRT(b.get(), "wedge (triangular prism)")) ++passed;
    }

    // A4: sphere at a non-origin centre
    {
        ++total;
        ON_Sphere sphere(ON_3dPoint(1.0, 2.0, 3.0), 2.5);
        std::unique_ptr<ON_Brep> b(ON_BrepSphere(sphere));
        if (TestBrepRT(b.get(), "off-origin sphere")) ++passed;
    }

    // A5: cylinder with non-unit radius and height
    {
        ++total;
        ON_Circle circle(ON_Plane::World_xy, 3.0);
        ON_Cylinder cyl(circle, 5.0);
        std::unique_ptr<ON_Brep> b(ON_BrepCylinder(cyl, true, true));
        if (TestBrepRT(b.get(), "large cylinder (r=3 h=5)")) ++passed;
    }

    // A6: torus with different major/minor radii
    {
        ++total;
        ON_Torus torus(ON_Plane::World_xy, 5.0, 2.0);
        std::unique_ptr<ON_Brep> b(ON_BrepTorus(torus));
        if (TestBrepRT(b.get(), "torus (R=5 r=2)")) ++passed;
    }

    // A7: quad-sphere (6 faces, no polar singularities)
    {
        ++total;
        std::unique_ptr<ON_Brep> b(ON_BrepQuadSphere(ON_3dPoint(0,0,0), 1.5));
        if (TestBrepRT(b.get(), "quad sphere (6 faces)")) ++passed;
    }

    // -----------------------------------------------------------------------
    // Group B: OCCT-native primitives → OCCTToON_Brep (no ON→OCCT step).
    // These exercise the analytical surface conversion fallback path.
    // -----------------------------------------------------------------------

    // B1: OCCT box (6 planar faces, 12 edges)
    {
        ++total;
        TopoDS_Shape s = BRepPrimAPI_MakeBox(2.0, 3.0, 4.0).Shape();
        if (TestOCCTNative(s, "OCCT native box", 6, 12)) ++passed;
    }

    // B2: OCCT full cone (conical + planar faces, 1 degenerate apex)
    {
        ++total;
        TopoDS_Shape s = BRepPrimAPI_MakeCone(1.0, 0.0, 2.0).Shape();
        if (TestOCCTNative(s, "OCCT native cone (full)", 2, 2)) ++passed;
    }

    // B3: OCCT truncated cone (frustum, 3 faces, no degenerate edges)
    {
        ++total;
        TopoDS_Shape s = BRepPrimAPI_MakeCone(0.5, 1.5, 3.0).Shape();
        if (TestOCCTNative(s, "OCCT native truncated cone", 3, 3)) ++passed;
    }

    // B4: OCCT sphere (1 face with 2 degenerate polar edges)
    {
        ++total;
        TopoDS_Shape s = BRepPrimAPI_MakeSphere(2.0).Shape();
        if (TestOCCTNative(s, "OCCT native sphere", 1, 1)) ++passed;
    }

    // B5: OCCT cylinder (3 faces: lateral + 2 caps)
    {
        ++total;
        TopoDS_Shape s = BRepPrimAPI_MakeCylinder(1.0, 3.0).Shape();
        if (TestOCCTNative(s, "OCCT native cylinder", 3, 3)) ++passed;
    }

    // -----------------------------------------------------------------------
    // Group C: Surface-accuracy spot-checks on round-tripped B-Reps.
    // We directly compare ON face sample points against the original
    // ON_Brep surface (before any OCCT step), ensuring geometric fidelity.
    // -----------------------------------------------------------------------

    // C1: box face accuracy — each face must reproduce exact planar points
    {
        ++total;
        ON_3dPoint corners[8];
        corners[0]=ON_3dPoint(0,0,0); corners[1]=ON_3dPoint(2,0,0);
        corners[2]=ON_3dPoint(2,3,0); corners[3]=ON_3dPoint(0,3,0);
        corners[4]=ON_3dPoint(0,0,4); corners[5]=ON_3dPoint(2,0,4);
        corners[6]=ON_3dPoint(2,3,4); corners[7]=ON_3dPoint(0,3,4);
        std::unique_ptr<ON_Brep> orig(ON_BrepBox(corners));

        TopoDS_Shape shape = open2open::ON_BrepToOCCT(*orig, kLinTol);
        ON_Brep rt;
        bool ok = open2open::OCCTToON_Brep(shape, rt, kLinTol);
        if (!ok || !rt.IsValid()) {
            std::printf("FAIL [box face accuracy]: round-trip failed\n");
        } else {
            // Verify that each face of the round-trip brep exactly
            // reproduces sample points from the original brep
            double max_err = 0.0;
            for (int fi = 0; fi < orig->m_F.Count(); ++fi) {
                const ON_Surface* os = orig->m_F[fi].SurfaceOf();
                const ON_Surface* rs = rt.m_F[fi].SurfaceOf();
                if (!os || !rs) continue;
                ON_Interval ou = os->Domain(0), ov = os->Domain(1);
                ON_Interval ru = rs->Domain(0), rv = rs->Domain(1);
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        double u0 = ou.Min() + ou.Length() * i / 2.0;
                        double v0 = ov.Min() + ov.Length() * j / 2.0;
                        double u1 = ru.Min() + ru.Length() * i / 2.0;
                        double v1 = rv.Min() + rv.Length() * j / 2.0;
                        ON_3dPoint p0, p1;
                        if (!os->EvPoint(u0, v0, p0)) continue;
                        if (!rs->EvPoint(u1, v1, p1)) continue;
                        double d = p0.DistanceTo(p1);
                        if (d > max_err) max_err = d;
                    }
                }
            }
            if (max_err > kGeomTol) {
                std::printf("FAIL [box face accuracy]: max error = %.3e\n", max_err);
            } else {
                std::printf("PASS [box face accuracy]: max surface error = %.3e\n",
                            max_err);
                ++passed;
            }
        }
    }

    // C2: sphere face accuracy
    {
        ++total;
        ON_Sphere sphere(ON_3dPoint(0,0,0), 1.0);
        std::unique_ptr<ON_Brep> orig(ON_BrepSphere(sphere));

        TopoDS_Shape shape = open2open::ON_BrepToOCCT(*orig, kLinTol);
        ON_Brep rt;
        bool ok = open2open::OCCTToON_Brep(shape, rt, kLinTol);
        if (!ok || !rt.IsValid()) {
            std::printf("FAIL [sphere face accuracy]: round-trip failed\n");
        } else {
            const ON_Surface* os = orig->m_F[0].SurfaceOf();
            const ON_Surface* rs =  rt.m_F[0].SurfaceOf();
            double max_err = 0.0;
            if (os && rs) {
                ON_Interval ou=os->Domain(0), ov=os->Domain(1);
                ON_Interval ru=rs->Domain(0), rv=rs->Domain(1);
                const int N = 8;
                for (int i = 1; i < N-1; ++i) {  // skip poles
                    for (int j = 0; j < N; ++j) {
                        double u0 = ou.Min() + ou.Length() * i / (N-1);
                        double v0 = ov.Min() + ov.Length() * j / (N-1);
                        double u1 = ru.Min() + ru.Length() * i / (N-1);
                        double v1 = rv.Min() + rv.Length() * j / (N-1);
                        ON_3dPoint p0, p1;
                        if (!os->EvPoint(u0, v0, p0)) continue;
                        if (!rs->EvPoint(u1, v1, p1)) continue;
                        double d = p0.DistanceTo(p1);
                        if (d > max_err) max_err = d;
                    }
                }
            }
            if (max_err > kGeomTol) {
                std::printf("FAIL [sphere face accuracy]: max error = %.3e\n", max_err);
            } else {
                std::printf("PASS [sphere face accuracy]: max surface error = %.3e\n",
                            max_err);
                ++passed;
            }
        }
    }

    std::printf("\nResult: %d / %d tests passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
