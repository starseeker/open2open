// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_surfaces.cpp — Round-trip tests for NURBS surface translation.
//
// Tests: ON_NurbsSurface → Geom_BSplineSurface → ON_NurbsSurface
//
// Verification strategy:
//   1. Structural checks (degrees, CV counts).
//   2. Grid sampling: compare points on original vs. round-tripped surface.

#include "open2open/geom_convert.h"

#include "opennurbs.h"

#include <Geom_BSplineSurface.hxx>
#include <gp_Pnt.hxx>

#include <cassert>
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
static bool TestSurfaceRoundTrip(const ON_NurbsSurface& original,
                                  const char* label)
{
    // --- ON → OCCT ---
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
        double d  = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (d > max_err) max_err = d;
    }
    if (max_err > kTol) {
        std::printf("FAIL [%s]: ON→OCCT geometric error = %.3e (tol %.3e)\n",
                    label, max_err, kTol);
        return false;
    }

    // --- OCCT → ON ---
    ON_NurbsSurface rt;
    if (!open2open::OCCTSurfaceToON(occt, rt)) {
        std::printf("FAIL [%s]: OCCTSurfaceToON failed\n", label);
        return false;
    }
    if (!rt.IsValid()) {
        std::printf("FAIL [%s]: round-trip surface is not valid\n", label);
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
        double d  = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (d > max_err) max_err = d;
    }
    if (max_err > kTol) {
        std::printf("FAIL [%s]: round-trip geometric error = %.3e (tol %.3e)\n",
                    label, max_err, kTol);
        return false;
    }

    std::printf("PASS [%s] (max round-trip error = %.3e)\n", label, max_err);
    return true;
}

// ---------------------------------------------------------------------------
// Build a bicubic non-rational patch over [0,1]x[0,1]
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeBicubicPatch()
{
    ON_NurbsSurface s;
    // order 4 in both directions, 4x4 CVs
    s.Create(3, false, 4, 4, 4, 4);

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double u = static_cast<double>(i) / 3.0;
            double v = static_cast<double>(j) / 3.0;
            s.SetCV(i, j,
                    ON_3dPoint(u, v, std::sin(u * ON_PI) * std::cos(v * ON_PI)));
        }
    }

    // Clamped knot vectors {0,0,0,1,1,1}
    for (int d = 0; d < 2; ++d) {
        s.m_knot[d][0] = 0.0; s.m_knot[d][1] = 0.0; s.m_knot[d][2] = 0.0;
        s.m_knot[d][3] = 1.0; s.m_knot[d][4] = 1.0; s.m_knot[d][5] = 1.0;
    }

    return s;
}

// ---------------------------------------------------------------------------
// Build a rational bilinear patch (four non-unit weights to make it rational)
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeRationalBilinear()
{
    ON_NurbsSurface s;
    // order 2 in both directions (degree 1), 2x2 CVs — simplest rational surface
    s.Create(3, true, 2, 2, 2, 2);

    // Corner weights: vary weights to exercise the rational code path
    double ws[2][2] = {{1.0, 0.8}, {0.8, 1.0}};
    double xs[2]    = {0.0, 1.0};
    double ys[2]    = {0.0, 1.0};
    double zs[2][2] = {{0.0, 0.5}, {0.5, 1.0}};

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            double w = ws[i][j];
            s.SetCV(i, j,
                    ON_4dPoint(xs[i] * w, ys[j] * w, zs[i][j] * w, w));
        }
    }

    // Clamped knot vectors {0,1} for degree-1 (order 2, 2 CVs → 2 knots)
    for (int d = 0; d < 2; ++d) {
        s.m_knot[d][0] = 0.0;
        s.m_knot[d][1] = 1.0;
    }

    return s;
}

// ---------------------------------------------------------------------------
// Build a U-periodic cylindrical surface (extruded arc)
// ---------------------------------------------------------------------------
static ON_NurbsSurface MakeCylindricalSurface()
{
    // Use openNURBS utility to build a cylinder NURBS
    ON_NurbsSurface s;
    ON_Cylinder cyl(ON_Circle(ON_Plane::World_xy, 1.0), 2.0);
    if (!cyl.GetNurbForm(s)) {
        // Fallback: return a placeholder
        s.Create(3, false, 2, 2, 2, 2);
    }
    return s;
}

int main()
{
    int passed = 0, total = 0;

    total += 3;

    ON_NurbsSurface bicubic    = MakeBicubicPatch();
    ON_NurbsSurface rational   = MakeRationalBilinear();
    ON_NurbsSurface cylindrical = MakeCylindricalSurface();

    if (TestSurfaceRoundTrip(bicubic,    "bicubic non-rational patch")) ++passed;
    if (TestSurfaceRoundTrip(rational,   "rational bilinear patch"))   ++passed;
    if (TestSurfaceRoundTrip(cylindrical,"cylindrical NURBS surface")) ++passed;

    std::printf("\nResult: %d / %d tests passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
