// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_curves_complex.cpp — Complex NURBS curve stress tests.
//
// Tests: ON_NurbsCurve → Geom_BSplineCurve → ON_NurbsCurve for:
//   - Full NURBS circle (rational, 9 CVs, degree 2)
//   - High-degree B-spline (degree 7)
//   - C1-kink cubic (degree 3 with interior knot multiplicity 2)
//   - Helical spiral (many CVs, non-uniform)
//   - Clamped quadratic with maximum weight variation

#include "open2open/geom_convert.h"

#include "opennurbs.h"

#include <Geom_BSplineCurve.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdio>
#include <vector>

static const double kTol = 1e-9;

// ---------------------------------------------------------------------------
// Helpers: uniform parameter sampling
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
static bool TestCurveRT(const ON_NurbsCurve& original, const char* label)
{
    // ON → OCCT
    Handle(Geom_BSplineCurve) occt = open2open::ON_NurbsCurveToOCCT(original);
    if (occt.IsNull()) {
        std::printf("FAIL [%s]: ON_NurbsCurveToOCCT returned null\n", label);
        return false;
    }
    if (occt->Degree() != original.m_order - 1) {
        std::printf("FAIL [%s]: degree mismatch (%d vs %d)\n",
                    label, occt->Degree(), original.m_order - 1);
        return false;
    }
    if (occt->NbPoles() != original.m_cv_count) {
        std::printf("FAIL [%s]: pole count mismatch (%d vs %d)\n",
                    label, occt->NbPoles(), original.m_cv_count);
        return false;
    }

    // Geometric sampling ON → OCCT
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
        double d  = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (d > max_err) max_err = d;
    }
    if (max_err > kTol) {
        std::printf("FAIL [%s]: ON→OCCT error = %.3e\n", label, max_err);
        return false;
    }

    // OCCT → ON
    ON_NurbsCurve rt;
    if (!open2open::OCCTCurveToON(occt, rt)) {
        std::printf("FAIL [%s]: OCCTCurveToON failed\n", label);
        return false;
    }
    if (!rt.IsValid()) {
        std::printf("FAIL [%s]: round-trip curve invalid\n", label);
        return false;
    }
    if (rt.m_order != original.m_order) {
        std::printf("FAIL [%s]: round-trip order %d != %d\n",
                    label, rt.m_order, original.m_order);
        return false;
    }

    // ON original vs round-trip geometric error
    std::vector<ON_3dPoint> rt_pts;
    SampleON(rt, N, rt_pts);
    max_err = 0.0;
    for (int i = 0; i < N; ++i) {
        double dx = on_pts[i].x - rt_pts[i].x;
        double dy = on_pts[i].y - rt_pts[i].y;
        double dz = on_pts[i].z - rt_pts[i].z;
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
// Build a full NURBS circle (radius 1, centre origin, standard 9-CV form)
// Degree 2, 9 CVs — the classical exact NURBS representation of a circle.
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeFullCircle()
{
    ON_NurbsCurve c;
    // degree 2 (order 3), 9 CVs — full circle with 3×3 clamped segments
    // knot vector length = order + cv_count - 2 = 3 + 9 - 2 = 10
    c.Create(3, true, 3, 9);

    const double w  = std::sqrt(2.0) / 2.0; // cos(45°)
    const double pi = ON_PI;

    // 8 segments of π/4 each; standard NURBS circle formulation
    // CV positions for a unit circle in the XY plane
    double angles[9] = {0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4, 3*pi/2, 7*pi/4, 2*pi};
    // Alternating unit-weight (on-circle) and w-weight (off-circle) CVs
    double weights[9] = {1, w, 1, w, 1, w, 1, w, 1};
    for (int k = 0; k < 9; ++k) {
        double wk = weights[k];
        double x  = std::cos(angles[k]);
        double y  = std::sin(angles[k]);
        // For the off-circle midpoint CVs (even indices after 0):
        // the actual control point is at the tangent midpoint
        if (k % 2 == 1) {
            // midpoint of arc: bisector direction
            double am = (angles[k-1] + angles[k+1]) / 2.0;
            x = std::cos(am) / w; // inflate by 1/w
            y = std::sin(am) / w;
        }
        c.SetCV(k, ON_4dPoint(x * wk, y * wk, 0.0, wk));
    }

    // Clamped knot vector for degree-2, 9 CVs: {0,0,1,1,2,2,3,3,4,4}
    // length = 10 = order + cv_count - 2
    double kv[10] = {0,0, 1,1, 2,2, 3,3, 4,4};
    for (int i = 0; i < 10; ++i)
        c.m_knot[i] = kv[i];

    return c;
}

// ---------------------------------------------------------------------------
// High-degree (degree 7) non-rational B-spline
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeHighDegreeCurve()
{
    ON_NurbsCurve c;
    // order 8 (degree 7), 12 CVs → knot count = 8+12-2 = 18
    // Clamped: order-1 = 7 repetitions at each end, 4 interior single knots
    c.Create(3, false, 8, 12);

    for (int i = 0; i < 12; ++i) {
        double t = static_cast<double>(i) / 11.0;
        c.SetCV(i, ON_3dPoint(t * 6.0,
                              std::sin(t * ON_PI * 3.0) * 0.5,
                              std::cos(t * ON_PI * 2.0) * 0.3));
    }

    // {0,0,0,0,0,0,0, 1,2,3,4, 5,5,5,5,5,5,5} — 7+4+7 = 18 ✓
    int ki = 0;
    for (int i = 0; i < 7; ++i) c.m_knot[ki++] = 0.0;
    c.m_knot[ki++] = 1.0; c.m_knot[ki++] = 2.0;
    c.m_knot[ki++] = 3.0; c.m_knot[ki++] = 4.0;
    for (int i = 0; i < 7; ++i) c.m_knot[ki++] = 5.0;

    return c;
}

// ---------------------------------------------------------------------------
// Cubic with a double interior knot (C1 kink at u=0.5)
// order 4 (degree 3), 8 CVs; knot: {0,0,0, 0.25,0.5,0.5, 0.75, 1,1,1} = 10 ✓
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeC1KinkCurve()
{
    ON_NurbsCurve c;
    c.Create(3, false, 4, 8);

    c.SetCV(0, ON_3dPoint(0.0, 0.0, 0.0));
    c.SetCV(1, ON_3dPoint(1.0, 2.0, 0.0));
    c.SetCV(2, ON_3dPoint(2.0, 2.0, 0.0));
    c.SetCV(3, ON_3dPoint(3.0, 1.0, 0.5)); // left side of kink
    c.SetCV(4, ON_3dPoint(3.0, 1.5, 0.5)); // right side of kink (same u)
    c.SetCV(5, ON_3dPoint(4.0, 2.0, 0.0));
    c.SetCV(6, ON_3dPoint(5.0, 1.0, 0.0));
    c.SetCV(7, ON_3dPoint(6.0, 0.0, 0.0));

    // {0,0,0, 0.25, 0.5,0.5, 0.75, 1,1,1} — 10 entries, double knot at 0.5
    c.m_knot[0] = 0.0; c.m_knot[1] = 0.0; c.m_knot[2] = 0.0;
    c.m_knot[3] = 0.25;
    c.m_knot[4] = 0.5;  c.m_knot[5] = 0.5;
    c.m_knot[6] = 0.75;
    c.m_knot[7] = 1.0;  c.m_knot[8] = 1.0;  c.m_knot[9] = 1.0;

    return c;
}

// ---------------------------------------------------------------------------
// Helical spiral — many CVs, non-trivial path
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeHelix()
{
    ON_NurbsCurve c;
    // Use openNURBS's built-in helix: not available directly, so build it
    // manually as a degree-3 approximation with 16 CVs.
    const int N = 16;
    // order 4, 16 CVs → 4+16-2 = 18 knots (clamped, uniform interior)
    c.Create(3, false, 4, N);

    for (int i = 0; i < N; ++i) {
        double t = static_cast<double>(i) / (N - 1); // 0..1
        double theta = t * 4.0 * ON_PI; // 2 full turns
        c.SetCV(i, ON_3dPoint(std::cos(theta),
                              std::sin(theta),
                              t * 3.0)); // rise of 3 units
    }

    // Clamped knot vector: {0,0,0, 1,2,...,12, 13,13,13}
    // length = 4+16-2 = 18
    // end multiplicity = 3, interior = 12 single knots
    int ki = 0;
    c.m_knot[ki++] = 0.0; c.m_knot[ki++] = 0.0; c.m_knot[ki++] = 0.0;
    for (int i = 1; i <= 12; ++i)
        c.m_knot[ki++] = static_cast<double>(i);
    c.m_knot[ki++] = 13.0; c.m_knot[ki++] = 13.0; c.m_knot[ki++] = 13.0;

    return c;
}

// ---------------------------------------------------------------------------
// Quadratic curve with extreme weight variation (high rationality stress)
// ---------------------------------------------------------------------------
static ON_NurbsCurve MakeWeightedQuadratic()
{
    ON_NurbsCurve c;
    // degree 2 (order 3), 7 CVs → 3+7-2 = 8 knots
    // Creates a piecewise-conic with 3 arcs, varying weights 1..10
    c.Create(3, true, 3, 7);

    double xs[7] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    double ys[7] = {0.0, 0.8, 0.0, 0.6, 0.0, 0.4, 0.0};
    double ws[7] = {1.0, 8.0, 1.0, 5.0, 1.0, 3.0, 1.0};
    for (int i = 0; i < 7; ++i)
        c.SetCV(i, ON_4dPoint(xs[i]*ws[i], ys[i]*ws[i], 0.0, ws[i]));

    // Clamped: {0,0, 1,1, 2,2, 3,3}
    c.m_knot[0] = 0.0; c.m_knot[1] = 0.0;
    c.m_knot[2] = 1.0; c.m_knot[3] = 1.0;
    c.m_knot[4] = 2.0; c.m_knot[5] = 2.0;
    c.m_knot[6] = 3.0; c.m_knot[7] = 3.0;

    return c;
}

int main()
{
    int passed = 0, total = 0;

    total += 5;
    if (TestCurveRT(MakeFullCircle(),      "full NURBS circle (9 CVs)"))   ++passed;
    if (TestCurveRT(MakeHighDegreeCurve(), "high-degree (deg 7) B-spline")) ++passed;
    if (TestCurveRT(MakeC1KinkCurve(),     "C1-kink cubic (double knot)"))  ++passed;
    if (TestCurveRT(MakeHelix(),           "helical spiral (16 CVs)"))      ++passed;
    if (TestCurveRT(MakeWeightedQuadratic(),"weighted quadratic (piecewise conic)")) ++passed;

    std::printf("\nResult: %d / %d tests passed\n", passed, total);
    return (passed == total) ? 0 : 1;
}
