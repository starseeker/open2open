// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_surfaces_complex.cpp — Complex NURBS surface stress tests.
//
// Tests: ON_NurbsSurface → Geom_BSplineSurface → ON_NurbsSurface for:
//   - Biquintic (degree 5 × 5) patch
//   - Mixed-degree surface (cubic × linear)
//   - Rational sphere surface (from ON_Sphere::GetNurbForm)
//   - Rational torus surface (from ON_Torus::GetNurbForm)
//   - High-resolution bicubic patch (8×8 control points)
//   - Surface with internal knots (C1 crease in u-direction)

#include "open2open/geom_convert.h"

#include "opennurbs.h"

#include <Geom_BSplineSurface.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdio>
#include <vector>

static const double kTol = 1e-9;

// ---------------------------------------------------------------------------
// Sampling helpers
// ---------------------------------------------------------------------------
static void SampleON(const ON_NurbsSurface& s, int nu, int nv,
                     std::vector<ON_3dPoint>& pts)
{
    pts.clear();
    pts.reserve(nu * nv);
    double u0 = s.Domain(0).Min(), u1 = s.Domain(0).Max();
    double v0 = s.Domain(1).Min(), v1 = s.Domain(1).Max();
    for (int i = 0; i < nu; ++i) {
        double u = u0 + (u1 - u0) * i / (nu - 1);
        for (int j = 0; j < nv; ++j) {
            double v = v0 + (v1 - v0) * j / (nv - 1);
            pts.push_back(s.PointAt(u, v));
        }
    }
}

static void SampleOCCT(const Handle(Geom_BSplineSurface)& s, int nu, int nv,
                        std::vector<gp_Pnt>& pts)
{
    pts.clear();
    pts.reserve(nu * nv);
    double u0, u1, v0, v1;
    s->Bounds(u0, u1, v0, v1);
    for (int i = 0; i < nu; ++i) {
        double u = u0 + (u1 - u0) * i / (nu - 1);
        for (int j = 0; j < nv; ++j) {
            double v = v0 + (v1 - v0) * j / (nv - 1);
            gp_Pnt p;
            s->D0(u, v, p);
            pts.push_back(p);
        }
    }
}

// ---------------------------------------------------------------------------
// Core round-trip test
// ---------------------------------------------------------------------------
static bool TestSurfaceRT(const ON_NurbsSurface& original, const char* label)
{
    // ON → OCCT
    Handle(Geom_BSplineSurface) occt =
        open2open::ON_NurbsSurfaceToOCCT(original);
    if (occt.IsNull()) {
        std::printf("FAIL [%s]: ON_NurbsSurfaceToOCCT returned null\n", label);
        return false;
    }
    if (occt->UDegree() != original.m_order[0] - 1) {
        std::printf("FAIL [%s]: U degree mismatch (%d vs %d)\n",
                    label, occt->UDegree(), original.m_order[0] - 1);
        return false;
    }
    if (occt->VDegree() != original.m_order[1] - 1) {
        std::printf("FAIL [%s]: V degree mismatch (%d vs %d)\n",
                    label, occt->VDegree(), original.m_order[1] - 1);
        return false;
    }
    if (occt->NbUPoles() != original.m_cv_count[0]) {
        std::printf("FAIL [%s]: U pole count mismatch (%d vs %d)\n",
                    label, occt->NbUPoles(), original.m_cv_count[0]);
        return false;
    }
    if (occt->NbVPoles() != original.m_cv_count[1]) {
        std::printf("FAIL [%s]: V pole count mismatch (%d vs %d)\n",
                    label, occt->NbVPoles(), original.m_cv_count[1]);
        return false;
    }

    const int NG = 10;
    std::vector<ON_3dPoint> on_pts;
    std::vector<gp_Pnt>     occt_pts;
    SampleON(original, NG, NG, on_pts);
    SampleOCCT(occt, NG, NG, occt_pts);

    double max_err = 0.0;
    for (int k = 0; k < NG * NG; ++k) {
        double dx = on_pts[k].x - occt_pts[k].X();
        double dy = on_pts[k].y - occt_pts[k].Y();
        double dz = on_pts[k].z - occt_pts[k].Z();
        double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d > max_err) max_err = d;
    }
    if (max_err > kTol) {
        std::printf("FAIL [%s]: ON→OCCT error = %.3e\n", label, max_err);
        return false;
    }

    // OCCT → ON
    ON_NurbsSurface rt;
    if (!open2open::OCCTSurfaceToON(occt, rt)) {
        std::printf("FAIL [%s]: OCCTSurfaceToON failed\n", label);
        return false;
    }
    if (!rt.IsValid()) {
        std::printf("FAIL [%s]: round-trip surface invalid\n", label);
        return false;
    }
    if (rt.m_order[0] != original.m_order[0] ||
        rt.m_order[1] != original.m_order[1]) {
        std::printf("FAIL [%s]: round-trip order mismatch\n", label);
        return false;
    }

    std::vector<ON_3dPoint> rt_pts;
    SampleON(rt, NG, NG, rt_pts);

    max_err = 0.0;
    for (int k = 0; k < NG * NG; ++k) {
        double dx = on_pts[k].x - rt_pts[k].x;
        double dy = on_pts[k].y - rt_pts[k].y;
        double dz = on_pts[k].z - rt_pts[k].z;
        double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d > max_err) max_err = d;
    }
    if (max_err > kTol) {
        std::printf("FAIL [%s]: round-trip error = %.3e\n", label, max_err);
        return false;
    }

    std::printf("PASS [%s] (max round-trip error = %.3e)\n", label, max_err);
    return true;
}

// ---------------------------------------------------------------------------
// Biquintic patch (order 6 × 6, 6×6 CVs)
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeBiquinticPatch()
{
    ON_NurbsSurface s;
    s.Create(3, false, 6, 6, 6, 6);

    for (int i = 0; i < 6; ++i) {
        double u = static_cast<double>(i) / 5.0;
        for (int j = 0; j < 6; ++j) {
            double v = static_cast<double>(j) / 5.0;
            // Wavy surface
            s.SetCV(i, j, ON_3dPoint(u, v,
                                     0.3 * std::sin(u * ON_PI * 2.0) *
                                           std::cos(v * ON_PI * 2.0)));
        }
    }

    // Clamped knot vectors for degree 5 (order 6), 6 CVs:
    // length = 6+6-2 = 10: {0,0,0,0,0,1,1,1,1,1}
    for (int d = 0; d < 2; ++d) {
        for (int k = 0; k < 5; ++k) s.m_knot[d][k] = 0.0;
        for (int k = 5; k < 10; ++k) s.m_knot[d][k] = 1.0;
    }

    return s;
}

// ---------------------------------------------------------------------------
// Mixed-degree surface: cubic × linear (order 4×2, 5×3 CVs)
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeMixedDegreeSurface()
{
    ON_NurbsSurface s;
    // order 4 in u, order 2 in v; 5 CVs in u, 3 CVs in v
    s.Create(3, false, 4, 2, 5, 3);

    for (int i = 0; i < 5; ++i) {
        double u = static_cast<double>(i) / 4.0;
        for (int j = 0; j < 3; ++j) {
            double v = static_cast<double>(j) / 2.0;
            s.SetCV(i, j, ON_3dPoint(u, v,
                                     0.2 * std::sin(u * ON_PI) * v));
        }
    }

    // U knot: order 4, 5 CVs → length = 4+5-2 = 7: {0,0,0,0.5,1,1,1}
    s.m_knot[0][0] = 0.0; s.m_knot[0][1] = 0.0; s.m_knot[0][2] = 0.0;
    s.m_knot[0][3] = 0.5;
    s.m_knot[0][4] = 1.0; s.m_knot[0][5] = 1.0; s.m_knot[0][6] = 1.0;

    // V knot: order 2, 3 CVs → length = 2+3-2 = 3: {0,0.5,1}
    s.m_knot[1][0] = 0.0;
    s.m_knot[1][1] = 0.5;
    s.m_knot[1][2] = 1.0;

    return s;
}

// ---------------------------------------------------------------------------
// Sphere NURBS surface (via ON_Sphere::GetNurbForm)
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeSphereSurface()
{
    ON_NurbsSurface s;
    ON_Sphere sphere(ON_3dPoint(0,0,0), 2.0);
    if (!sphere.GetNurbForm(s)) {
        // Fallback — should never happen
        s.Create(3, false, 2, 2, 2, 2);
    }
    return s;
}

// ---------------------------------------------------------------------------
// Torus NURBS surface (via ON_Torus::GetNurbForm)
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeTorusSurface()
{
    ON_NurbsSurface s;
    ON_Torus torus(ON_Plane::World_xy, 4.0, 1.5);
    if (!torus.GetNurbForm(s)) {
        s.Create(3, false, 2, 2, 2, 2);
    }
    return s;
}

// ---------------------------------------------------------------------------
// High-resolution bicubic patch (8×8 CVs)
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeHighResBicubic()
{
    ON_NurbsSurface s;
    const int N = 8;
    s.Create(3, false, 4, 4, N, N);

    for (int i = 0; i < N; ++i) {
        double u = static_cast<double>(i) / (N - 1);
        for (int j = 0; j < N; ++j) {
            double v = static_cast<double>(j) / (N - 1);
            // Saddle-like surface with multiple humps
            s.SetCV(i, j, ON_3dPoint(u, v,
                                     std::sin(u * 3.0 * ON_PI) *
                                     std::cos(v * 3.0 * ON_PI) * 0.15));
        }
    }

    // U/V knot: order 4, N CVs → N+2 interior knots
    // length = 4+N-2 = N+2 = 10 for N=8
    // Clamped uniform: {0,0,0, 1,2,3,4,5, 6,6,6} = 11... let me recalculate
    // 4+8-2 = 10. With clamped ends (mult=3) and interior single knots:
    // {0,0,0, 1,2,3,4, 5,5,5} = 10 ✓
    for (int d = 0; d < 2; ++d) {
        s.m_knot[d][0] = 0.0; s.m_knot[d][1] = 0.0; s.m_knot[d][2] = 0.0;
        for (int k = 0; k < N - 4; ++k)
            s.m_knot[d][3 + k] = static_cast<double>(k + 1);
        int end = 3 + (N - 4);
        s.m_knot[d][end]   = static_cast<double>(N - 3);
        s.m_knot[d][end+1] = static_cast<double>(N - 3);
        s.m_knot[d][end+2] = static_cast<double>(N - 3);
    }

    return s;
}

// ---------------------------------------------------------------------------
// Surface with a C1 crease in the U direction (double interior knot)
// Bicubic 7×4 CVs with a double knot at u=0.5
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeCreasSurface()
{
    ON_NurbsSurface s;
    // order 4×4, 7×4 CVs
    // U knots: 4+7-2=9; with C1 crease at 0.5:
    //   {0,0,0, 0.25, 0.5,0.5, 0.75, 1,1,1} = 10 ≠ 9
    // Hmm. For 7 CVs, order 4: m_knot_length = 4+7-2 = 9.
    // Clamped knots: 3 at each end + 3 interior = 9.
    // Single crease at 0.5 (mult 2): {0,0,0, 0.5,0.5, 1,1,1} = 8 < 9.
    // We need one more interior knot: {0,0,0, 0.25, 0.5,0.5, 1,1,1} = 9 ✓
    s.Create(3, false, 4, 4, 7, 4);

    for (int i = 0; i < 7; ++i) {
        double u = static_cast<double>(i) / 6.0;
        for (int j = 0; j < 4; ++j) {
            double v = static_cast<double>(j) / 3.0;
            double z = (u < 0.5) ? (u * 0.5) : ((1.0 - u) * 0.5);
            s.SetCV(i, j, ON_3dPoint(u, v, z * std::sin(v * ON_PI)));
        }
    }

    // U knot: {0,0,0, 0.25, 0.5,0.5, 1,1,1} = 9 ✓
    s.m_knot[0][0] = 0.0; s.m_knot[0][1] = 0.0; s.m_knot[0][2] = 0.0;
    s.m_knot[0][3] = 0.25;
    s.m_knot[0][4] = 0.5;  s.m_knot[0][5] = 0.5;
    s.m_knot[0][6] = 1.0;  s.m_knot[0][7] = 1.0; s.m_knot[0][8] = 1.0;

    // V knot: order 4, 4 CVs → 4+4-2=6: {0,0,0,1,1,1}
    s.m_knot[1][0] = 0.0; s.m_knot[1][1] = 0.0; s.m_knot[1][2] = 0.0;
    s.m_knot[1][3] = 1.0; s.m_knot[1][4] = 1.0; s.m_knot[1][5] = 1.0;

    return s;
}

int main()
{
    int passed = 0, total = 0;

    total += 6;
    if (TestSurfaceRT(MakeBiquinticPatch(),    "biquintic patch (deg 5×5)"))      ++passed;
    if (TestSurfaceRT(MakeMixedDegreeSurface(),"mixed-degree surface (deg 3×1)")) ++passed;
    if (TestSurfaceRT(MakeSphereSurface(),     "sphere NURBS (from GetNurbForm)")) ++passed;
    if (TestSurfaceRT(MakeTorusSurface(),      "torus NURBS (from GetNurbForm)"))  ++passed;
    if (TestSurfaceRT(MakeHighResBicubic(),    "high-res bicubic (8×8 CVs)"))      ++passed;
    if (TestSurfaceRT(MakeCreasSurface(),      "bicubic with C1 crease (double knot)")) ++passed;

    std::printf("\nResult: %d / %d tests passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
