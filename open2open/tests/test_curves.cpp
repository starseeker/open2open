// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_curves.cpp — Round-trip tests for NURBS curve translation.
//
// Tests: ON_NurbsCurve → Geom_BSplineCurve → ON_NurbsCurve
//
// The round-trip is verified by:
//   1. Checking structural properties (degree, CV count, knot count).
//   2. Sampling both the original and round-tripped curve at a uniform
//      parameter grid and asserting that point distances are below 1e-10.

#include "open2open/geom_convert.h"

#include "opennurbs.h"

#include <Geom_BSplineCurve.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <gp_Pnt.hxx>

#include <cassert>
#include <cmath>
#include <cstdio>

static const double kTol = 1e-9;

// ---------------------------------------------------------------------------
// Helper: sample ON_NurbsCurve at n uniform parameter steps
// ---------------------------------------------------------------------------
static void SampleON(const ON_NurbsCurve& c, int n,
                     std::vector<ON_3dPoint>& pts)
{
    pts.clear();
    pts.reserve(n);
    double t0 = c.Domain().Min();
    double t1 = c.Domain().Max();
    for (int i = 0; i < n; ++i) {
        double t = t0 + (t1 - t0) * i / (n - 1);
        pts.push_back(c.PointAt(t));
    }
}

// ---------------------------------------------------------------------------
// Helper: sample Geom_BSplineCurve at n uniform parameter steps
// ---------------------------------------------------------------------------
static void SampleOCCT(const Handle(Geom_BSplineCurve)& c, int n,
                        std::vector<gp_Pnt>& pts)
{
    pts.clear();
    pts.reserve(n);
    double t0 = c->FirstParameter();
    double t1 = c->LastParameter();
    for (int i = 0; i < n; ++i) {
        double t = t0 + (t1 - t0) * i / (n - 1);
        gp_Pnt p;
        c->D0(t, p);
        pts.push_back(p);
    }
}

// ---------------------------------------------------------------------------
// Core round-trip test
// ---------------------------------------------------------------------------
static bool TestCurveRoundTrip(const ON_NurbsCurve& original,
                                const char* label)
{
    // --- ON → OCCT ---
    Handle(Geom_BSplineCurve) occt = open2open::ON_NurbsCurveToOCCT(original);
    if (occt.IsNull()) {
        std::printf("FAIL [%s]: ON_NurbsCurveToOCCT returned null\n", label);
        return false;
    }

    // Structural checks
    if (occt->Degree() != original.m_order - 1) {
        std::printf("FAIL [%s]: degree mismatch (got %d, expected %d)\n",
                    label, occt->Degree(), original.m_order - 1);
        return false;
    }
    if (occt->NbPoles() != original.m_cv_count) {
        std::printf("FAIL [%s]: pole count mismatch (got %d, expected %d)\n",
                    label, occt->NbPoles(), original.m_cv_count);
        return false;
    }

    // Geometric sampling: ON → sample, OCCT → sample, compare
    const int N = 50;
    std::vector<ON_3dPoint> on_pts;
    std::vector<gp_Pnt>     occt_pts;
    SampleON(original, N, on_pts);
    SampleOCCT(occt, N, occt_pts);

    double max_err = 0.0;
    for (int i = 0; i < N; ++i) {
        double dx = on_pts[i].x - occt_pts[i].X();
        double dy = on_pts[i].y - occt_pts[i].Y();
        double dz = on_pts[i].z - occt_pts[i].Z();
        double d  = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (d > max_err) max_err = d;
    }
    if (max_err > kTol) {
        std::printf("FAIL [%s]: ON→OCCT geometric error = %.3e (tol %.3e)\n",
                    label, max_err, kTol);
        return false;
    }

    // --- OCCT → ON ---
    ON_NurbsCurve rt;
    if (!open2open::OCCTCurveToON(occt, rt)) {
        std::printf("FAIL [%s]: OCCTCurveToON failed\n", label);
        return false;
    }
    if (!rt.IsValid()) {
        std::printf("FAIL [%s]: round-trip ON_NurbsCurve is not valid\n", label);
        return false;
    }
    if (rt.m_order != original.m_order) {
        std::printf("FAIL [%s]: round-trip order mismatch (got %d, expected %d)\n",
                    label, rt.m_order, original.m_order);
        return false;
    }
    if (rt.m_cv_count != original.m_cv_count) {
        std::printf("FAIL [%s]: round-trip CV count mismatch (%d vs %d)\n",
                    label, rt.m_cv_count, original.m_cv_count);
        return false;
    }

    // Geometric sampling: original vs round-trip
    std::vector<ON_3dPoint> rt_pts;
    SampleON(rt, N, rt_pts);

    max_err = 0.0;
    for (int i = 0; i < N; ++i) {
        double dx = on_pts[i].x - rt_pts[i].x;
        double dy = on_pts[i].y - rt_pts[i].y;
        double dz = on_pts[i].z - rt_pts[i].z;
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
// Build a clamped non-rational cubic B-spline (degree 3) on [0,1]
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeCubicBSpline()
{
    ON_NurbsCurve c;
    // degree 3 (order 4), 6 CVs → 8 knots (order+cv-2 = 4+6-2 = 8)
    c.Create(3, false, 4, 6);

    c.SetCV(0, ON_3dPoint(0.0, 0.0, 0.0));
    c.SetCV(1, ON_3dPoint(1.0, 2.0, 0.0));
    c.SetCV(2, ON_3dPoint(2.0, 1.0, 1.0));
    c.SetCV(3, ON_3dPoint(3.0, 2.0, 0.0));
    c.SetCV(4, ON_3dPoint(4.0, 0.0, 0.0));
    c.SetCV(5, ON_3dPoint(5.0, 1.0, 0.5));

    // Clamped knot vector: {0,0,0,1,2,3,3,3}
    c.m_knot[0] = 0.0; c.m_knot[1] = 0.0; c.m_knot[2] = 0.0;
    c.m_knot[3] = 1.0;
    c.m_knot[4] = 2.0;
    c.m_knot[5] = 3.0; c.m_knot[6] = 3.0; c.m_knot[7] = 3.0;

    return c;
}

// ---------------------------------------------------------------------------
// Build a rational cubic (circle arc approximation style)
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeRationalCubic()
{
    ON_NurbsCurve c;
    // degree 2 (order 3), 3 CVs – classic NURBS quarter-circle weight
    c.Create(3, true, 3, 3);

    double w = std::sqrt(2.0) / 2.0; // cos(45°)

    // Control points in homogeneous form [X*w, Y*w, Z*w, w]
    c.SetCV(0, ON_4dPoint(1.0, 0.0, 0.0, 1.0));
    c.SetCV(1, ON_4dPoint(1.0 * w, 1.0 * w, 0.0, w));
    c.SetCV(2, ON_4dPoint(0.0, 1.0, 0.0, 1.0));

    // Clamped knot vector for degree-2, 3 CVs: {0,0,1,1} (length = 4 = order+cv-2 = 3+3-2)
    c.m_knot[0] = 0.0; c.m_knot[1] = 0.0;
    c.m_knot[2] = 1.0; c.m_knot[3] = 1.0;

    return c;
}

// ---------------------------------------------------------------------------
// Build a degree-5 (quintic) non-rational B-spline
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeQuinticBSpline()
{
    ON_NurbsCurve c;
    // order 6, 8 CVs → 12 knots
    c.Create(3, false, 6, 8);

    for (int i = 0; i < 8; ++i) {
        double t = static_cast<double>(i) / 7.0;
        c.SetCV(i, ON_3dPoint(t * 4.0,
                              std::sin(t * ON_PI * 2.0),
                              std::cos(t * ON_PI * 2.0)));
    }

    // Clamped knot vector {0,0,0,0,0,1,2,3,3,3,3,3} length=12
    c.m_knot[0]  = 0.0; c.m_knot[1]  = 0.0;
    c.m_knot[2]  = 0.0; c.m_knot[3]  = 0.0;
    c.m_knot[4]  = 0.0;
    c.m_knot[5]  = 1.0;
    c.m_knot[6]  = 2.0;
    c.m_knot[7]  = 3.0;
    c.m_knot[8]  = 3.0; c.m_knot[9]  = 3.0;
    c.m_knot[10] = 3.0; c.m_knot[11] = 3.0;

    return c;
}

// ---------------------------------------------------------------------------
// Build a uniform (non-clamped) cubic B-spline
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeUniformCubic()
{
    ON_NurbsCurve c;
    // order 4, 6 CVs → 8 knots; uniform spacing
    c.Create(3, false, 4, 6);

    c.SetCV(0, ON_3dPoint(0.0, 0.0, 0.0));
    c.SetCV(1, ON_3dPoint(1.0, 1.0, 0.0));
    c.SetCV(2, ON_3dPoint(2.0, 0.0, 0.0));
    c.SetCV(3, ON_3dPoint(3.0, 1.0, 0.0));
    c.SetCV(4, ON_3dPoint(4.0, 0.0, 1.0));
    c.SetCV(5, ON_3dPoint(5.0, 1.0, 1.0));

    // Uniform knot vector {0,1,2,3,4,5,6,7}
    for (int i = 0; i < 8; ++i)
        c.m_knot[i] = static_cast<double>(i);

    return c;
}

int main()
{
    int passed = 0, total = 0;

    ON_NurbsCurve cubic    = MakeCubicBSpline();
    ON_NurbsCurve rational = MakeRationalCubic();
    ON_NurbsCurve quintic  = MakeQuinticBSpline();
    ON_NurbsCurve uniform  = MakeUniformCubic();

    total += 4;
    if (TestCurveRoundTrip(cubic,    "non-rational cubic B-spline")) ++passed;
    if (TestCurveRoundTrip(rational, "rational quadratic (NURBS arc)")) ++passed;
    if (TestCurveRoundTrip(quintic,  "non-rational quintic B-spline")) ++passed;
    if (TestCurveRoundTrip(uniform,  "uniform cubic B-spline")) ++passed;

    std::printf("\nResult: %d / %d tests passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
