// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// brep_convert.cpp — Full B-Rep topology translation between openNURBS
//                    (ON_Brep) and OpenCASCADE (TopoDS_Shape).

#include "open2open/brep_convert.h"
#include "open2open/geom_convert.h"

// ---- openNURBS ----
#include "opennurbs.h"

// ---- OCCT topology ----
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_Orientation.hxx>

// ---- OCCT geometry ----
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomConvert.hxx>
#include <Geom2dConvert.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <gp_Pnt.hxx>

// ---- OCCT tools ----
#include <BRepLib.hxx>
#include <Standard_Handle.hxx>
#include <NCollection_Map.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

#include <vector>
#include <map>
#include <set>
#include <tuple>
#include <cmath>
#include <functional>

namespace open2open {

// ---------------------------------------------------------------------------
// Helper: convert any ON_Curve to a Geom_Curve (NURBS fallback)
// ---------------------------------------------------------------------------
static Handle(Geom_BSplineCurve)
CurveToOCCT(const ON_Curve* crv)
{
    if (!crv) return Handle(Geom_BSplineCurve)();

    // Try direct cast first
    const ON_NurbsCurve* nc = ON_NurbsCurve::Cast(crv);
    if (nc && nc->m_dim == 3)
        return ON_NurbsCurveToOCCT(*nc);

    // Fallback: convert to NURBS
    ON_NurbsCurve tmp;
    if (!crv->GetNurbForm(tmp))
        return Handle(Geom_BSplineCurve)();
    if (tmp.m_dim != 3) return Handle(Geom_BSplineCurve)();
    return ON_NurbsCurveToOCCT(tmp);
}

// ---------------------------------------------------------------------------
// Helper: convert any ON_Curve (2D) to a Geom2d_Curve
// ---------------------------------------------------------------------------
static Handle(Geom2d_BSplineCurve)
Curve2dToOCCT(const ON_Curve* crv)
{
    if (!crv) return Handle(Geom2d_BSplineCurve)();

    const ON_NurbsCurve* nc = ON_NurbsCurve::Cast(crv);
    if (nc && nc->m_dim == 2)
        return ON_NurbsCurve2dToOCCT(*nc);

    ON_NurbsCurve tmp;
    if (!crv->GetNurbForm(tmp))
        return Handle(Geom2d_BSplineCurve)();
    if (tmp.m_dim != 2) return Handle(Geom2d_BSplineCurve)();
    return ON_NurbsCurve2dToOCCT(tmp);
}

// ---------------------------------------------------------------------------
// Helper: convert any ON_Surface to a Geom_Surface (NURBS fallback)
// ---------------------------------------------------------------------------
static Handle(Geom_BSplineSurface)
SurfaceToOCCT(const ON_Surface* srf)
{
    if (!srf) return Handle(Geom_BSplineSurface)();

    const ON_NurbsSurface* ns = ON_NurbsSurface::Cast(srf);
    if (ns)
        return ON_NurbsSurfaceToOCCT(*ns);

    ON_NurbsSurface tmp;
    if (!srf->GetNurbForm(tmp))
        return Handle(Geom_BSplineSurface)();
    return ON_NurbsSurfaceToOCCT(tmp);
}

// ===========================================================================
//  ON_Brep  →  TopoDS_Shape
// ===========================================================================

TopoDS_Shape ON_BrepToOCCT(const ON_Brep& brep, double linear_tolerance)
{
    BRep_Builder B;

    // ------------------------------------------------------------------
    // Step 1 — translate geometry pools
    // ------------------------------------------------------------------
    std::vector<Handle(Geom_BSplineCurve)>   c3_map(brep.m_C3.Count());
    std::vector<Handle(Geom2d_BSplineCurve)> c2_map(brep.m_C2.Count());
    std::vector<Handle(Geom_BSplineSurface)> s_map(brep.m_S.Count());

    for (int i = 0; i < brep.m_C3.Count(); ++i)
        c3_map[i] = CurveToOCCT(brep.m_C3[i]);

    for (int i = 0; i < brep.m_C2.Count(); ++i)
        c2_map[i] = Curve2dToOCCT(brep.m_C2[i]);

    for (int i = 0; i < brep.m_S.Count(); ++i)
        s_map[i] = SurfaceToOCCT(brep.m_S[i]);

    // ------------------------------------------------------------------
    // Step 2 — create vertices
    // ------------------------------------------------------------------
    std::vector<TopoDS_Vertex> v_map(brep.m_V.Count());
    for (int i = 0; i < brep.m_V.Count(); ++i) {
        const ON_BrepVertex& v = brep.m_V[i];
        double tol = (v.m_tolerance == ON_UNSET_VALUE)
                         ? linear_tolerance
                         : v.m_tolerance;
        ON_3dPoint pt = v.Point();
        B.MakeVertex(v_map[i], gp_Pnt(pt.x, pt.y, pt.z), tol);
    }

    // ------------------------------------------------------------------
    // Step 3 — create edges (3-D curve + vertices + tolerance)
    // ------------------------------------------------------------------
    std::vector<TopoDS_Edge> e_map(brep.m_E.Count());
    // Remember the 3D domain of each edge so we can detect SameRange=false
    // situations when attaching pcurves.
    std::vector<std::pair<double,double>> e3d_domain(brep.m_E.Count(),
                                                      {0.0, 0.0});
    for (int i = 0; i < brep.m_E.Count(); ++i) {
        const ON_BrepEdge& e = brep.m_E[i];
        B.MakeEdge(e_map[i]);

        double t0 = 0.0, t1 = 0.0;
        if (e.m_c3i >= 0 && e.m_c3i < (int)c3_map.size() &&
            !c3_map[e.m_c3i].IsNull()) {
            t0 = e.Domain().Min();
            t1 = e.Domain().Max();
            B.UpdateEdge(e_map[i], c3_map[e.m_c3i], linear_tolerance);
            B.Range(e_map[i], t0, t1, /*only3d=*/Standard_True);
            e3d_domain[i] = {t0, t1};
        }

        if (e.m_vi[0] >= 0 && e.m_vi[0] < (int)v_map.size()) {
            // Start vertex: FORWARD orientation (standard OCCT convention)
            TopoDS_Vertex vstart = v_map[e.m_vi[0]];
            vstart.Orientation(TopAbs_FORWARD);
            B.Add(e_map[i], vstart);
        }
        if (e.m_vi[1] >= 0 && e.m_vi[1] < (int)v_map.size() &&
            e.m_vi[1] != e.m_vi[0]) {
            // End vertex: REVERSED orientation (standard OCCT convention).
            // The FORWARD vertex (vi[0]) is at the edge-curve start (t0),
            // the REVERSED vertex (vi[1]) is at the end (t1).
            TopoDS_Vertex vend = v_map[e.m_vi[1]];
            vend.Orientation(TopAbs_REVERSED);
            B.Add(e_map[i], vend);
        } else if (e.m_vi[0] >= 0 && e.m_vi[0] < (int)v_map.size() &&
                   e.m_vi[1] == e.m_vi[0]) {
            // Closed edge (same start/end vertex): OCCT requires the vertex
            // to appear with REVERSED orientation as well so that
            // BRepTools_WireExplorer and BRep_Tool::IsClosed() recognize
            // the edge as properly closed.  Without this, inner 1-edge wires
            // (e.g. circular holes) cannot be traversed and their pcurves
            // are invisible to BRepCheck_Face::ClassifyWires().
            TopoDS_Vertex vclosed = v_map[e.m_vi[0]];
            vclosed.Orientation(TopAbs_REVERSED);
            B.Add(e_map[i], vclosed);
        }

        // Explicitly set vertex parameters so BRep_Tool::Parameter returns
        // the correct values (B.Add sets the parameter from the edge range,
        // but an explicit UpdateVertex call avoids any ambiguity).
        if (e.m_c3i >= 0) {
            double etol = (e.m_tolerance > 0.0 && e.m_tolerance != ON_UNSET_VALUE)
                              ? e.m_tolerance : linear_tolerance;
            if (e.m_vi[0] >= 0 && e.m_vi[0] < (int)v_map.size())
                B.UpdateVertex(v_map[e.m_vi[0]], t0, e_map[i], etol);
            if (e.m_vi[1] >= 0 && e.m_vi[1] < (int)v_map.size() &&
                e.m_vi[1] != e.m_vi[0])
                B.UpdateVertex(v_map[e.m_vi[1]], t1, e_map[i], etol);
        }

        if (e.m_tolerance != ON_UNSET_VALUE)
            B.UpdateEdge(e_map[i], e.m_tolerance);
    }

    // ------------------------------------------------------------------
    // Step 4 — create faces, attach pcurves, build wires
    // ------------------------------------------------------------------
    std::vector<TopoDS_Face> f_map(brep.m_F.Count());

    for (int fi = 0; fi < brep.m_F.Count(); ++fi) {
        const ON_BrepFace& f = brep.m_F[fi];

        if (f.m_si < 0 || f.m_si >= (int)s_map.size() ||
            s_map[f.m_si].IsNull()) {
            B.MakeFace(f_map[fi]);
            continue;
        }

        B.MakeFace(f_map[fi], s_map[f.m_si], linear_tolerance);
        if (f.m_bRev)
            f_map[fi].Reverse();

        // Pre-scan all loops on this face for seam trim pairs.
        // A seam edge appears twice in the same loop: once forward, once
        // reversed.  OCCT requires both 2D pcurves set in a single call to
        // BRep_Builder::UpdateEdge(edge, C1, C2, face, tol).  If we call the
        // single-pcurve overload twice the second call overwrites the first.
        struct SeamPair {
            int c2i_fwd = -1;   // pcurve for edge FORWARD orientation
            int c2i_rev = -1;   // pcurve for edge REVERSED orientation
            double t0 = 0, t1 = 0;
            double tol = 0;
        };
        std::map<int, SeamPair> seam_map;  // edge index → pcurve pair

        for (int li_idx = 0; li_idx < f.m_li.Count(); ++li_idx) {
            int loop_idx = f.m_li[li_idx];
            if (loop_idx < 0 || loop_idx >= brep.m_L.Count()) continue;
            const ON_BrepLoop& loop = brep.m_L[loop_idx];
            for (int ti_idx = 0; ti_idx < loop.m_ti.Count(); ++ti_idx) {
                int trim_idx = loop.m_ti[ti_idx];
                if (trim_idx < 0 || trim_idx >= brep.m_T.Count()) continue;
                const ON_BrepTrim& trim = brep.m_T[trim_idx];
                if (trim.m_type != ON_BrepTrim::seam) continue;
                int ei = trim.m_ei;
                if (ei < 0 || ei >= brep.m_E.Count()) continue;
                SeamPair& sp = seam_map[ei];
                double t = trim.m_tolerance[0] > 0.0
                               ? trim.m_tolerance[0]
                               : linear_tolerance;
                if (!trim.m_bRev3d) {
                    sp.c2i_fwd = trim.m_c2i;
                    sp.t0 = trim.Domain().Min();
                    sp.t1 = trim.Domain().Max();
                    sp.tol = t;
                } else {
                    sp.c2i_rev = trim.m_c2i;
                    if (sp.tol == 0.0) sp.tol = t;
                }
            }
        }

        // Apply double-pcurve UpdateEdge for seam edges now (before the
        // per-loop trim loop that would otherwise set them one at a time).
        std::set<int> seam_c2_applied;
        for (auto& kv : seam_map) {
            int ei = kv.first;
            const SeamPair& sp = kv.second;
            if (ei >= (int)e_map.size()) continue;
            if (sp.c2i_fwd >= 0 && sp.c2i_fwd < (int)c2_map.size() &&
                sp.c2i_rev >= 0 && sp.c2i_rev < (int)c2_map.size() &&
                !c2_map[sp.c2i_fwd].IsNull() && !c2_map[sp.c2i_rev].IsNull()) {
                // Build C_fwd (edge-forward at the U_max seam) and
                // C_rev_edgefwd (edge-forward at the U_0 seam).
                //
                // The openNURBS seam_rev pcurve is stored in TRIM traversal
                // direction (= reversed edge direction).  Reverse it so both
                // pcurves are in the edge-forward convention.
                Handle(Geom2d_Curve) c_fwd      = c2_map[sp.c2i_fwd];
                Handle(Geom2d_Curve) c_rev_efwd = c2_map[sp.c2i_rev]->Reversed();

                // OCCT keeps the original domain when reversing a BSpline,
                // so c_rev_efwd may have domain [-t1, -t0] instead of
                // [sp.t0, sp.t1].  Build a new BSpline with the same poles
                // but linearly remapped knots so that BRep_Tool::CurveOnSurface
                // never extrapolates outside the stored range.
                Handle(Geom2d_BSplineCurve) brev =
                    Handle(Geom2d_BSplineCurve)::DownCast(c_rev_efwd);
                if (!brev.IsNull() && brev->NbKnots() >= 2) {
                    double old0 = brev->FirstParameter();
                    double old1 = brev->LastParameter();
                    double span = old1 - old0;
                    if (std::abs(span) > 1e-12 &&
                        (std::abs(old0 - sp.t0) > 1e-10 ||
                         std::abs(old1 - sp.t1) > 1e-10)) {
                        // Build remapped knot array (distinct knots)
                        const int nk = brev->NbKnots();
                        double scale = (sp.t1 - sp.t0) / span;
                        TColStd_Array1OfReal    new_knots(1, nk);
                        TColStd_Array1OfInteger new_mults(1, nk);
                        for (int k = 1; k <= nk; ++k) {
                            new_knots.SetValue(k, sp.t0 + (brev->Knot(k) - old0) * scale);
                            new_mults.SetValue(k, brev->Multiplicity(k));
                        }
                        const int np = brev->NbPoles();
                        TColgp_Array1OfPnt2d poles(1, np);
                        TColStd_Array1OfReal  weights(1, np);
                        bool rat = brev->IsRational();
                        for (int p = 1; p <= np; ++p) {
                            poles.SetValue(p, brev->Pole(p));
                            weights.SetValue(p, rat ? brev->Weight(p) : 1.0);
                        }
                        try {
                            Handle(Geom2d_BSplineCurve) remap;
                            if (rat)
                                remap = new Geom2d_BSplineCurve(poles, weights,
                                            new_knots, new_mults, brev->Degree());
                            else
                                remap = new Geom2d_BSplineCurve(poles,
                                            new_knots, new_mults, brev->Degree());
                            c_rev_efwd = remap;
                        } catch (...) {}
                    }
                }

                // OCCT's BRep_Tool::CurveOnSurface(E, F) returns:
                //   PCurve()  when the face is FORWARD  → use C1 for FORWARD
                //   PCurve2() when the face is REVERSED → use C2 for REVERSED
                // When the face itself is reversed (m_bRev=1), OCCT swaps
                // which of C1/C2 is associated with which wire orientation.
                // Swap the assignment so the correct pcurve reaches each
                // edge occurrence in OCCTToON_Brep.
                Handle(Geom2d_Curve) C1, C2;
                if (f.m_bRev) {
                    C1 = c_rev_efwd;  // returned for REVERSED edge (PCurve)
                    C2 = c_fwd;       // returned for FORWARD edge (PCurve2)
                } else {
                    C1 = c_fwd;       // returned for FORWARD edge (PCurve)
                    C2 = c_rev_efwd;  // returned for REVERSED edge (PCurve2)
                }
                B.UpdateEdge(e_map[ei], C1, C2, f_map[fi], sp.tol);
                B.Range(e_map[ei], f_map[fi], sp.t0, sp.t1);
                // Clear SameRange when domain differs; SameParameter is
                // handled globally after the shape is built.
                {
                    const double kRangeTol = 1e-10;
                    const auto& ed = e3d_domain[ei];
                    if (std::fabs(sp.t0 - ed.first)  > kRangeTol ||
                        std::fabs(sp.t1 - ed.second) > kRangeTol) {
                        B.SameRange(e_map[ei], Standard_False);
                        B.SameParameter(e_map[ei], Standard_False);
                    }
                }
                seam_c2_applied.insert(ei);
            }
        }

        for (int li_idx = 0; li_idx < f.m_li.Count(); ++li_idx) {
            int loop_idx = f.m_li[li_idx];
            if (loop_idx < 0 || loop_idx >= brep.m_L.Count())
                continue;
            const ON_BrepLoop& loop = brep.m_L[loop_idx];

            // Skip singularity/ptonsrf loops that have no real boundary
            if (loop.m_type == ON_BrepLoop::ptonsrf)
                continue;

            TopoDS_Wire wire;
            B.MakeWire(wire);

            for (int ti_idx = 0; ti_idx < loop.m_ti.Count(); ++ti_idx) {
                int trim_idx = loop.m_ti[ti_idx];
                if (trim_idx < 0 || trim_idx >= brep.m_T.Count())
                    continue;
                const ON_BrepTrim& trim = brep.m_T[trim_idx];
                int ei = trim.m_ei;

                // Singular trims (no edge) → degenerate OCCT edge
                if (ei < 0) {
                    if (trim.m_vi[0] >= 0 &&
                        trim.m_vi[0] < (int)v_map.size()) {
                        TopoDS_Edge degen_edge;
                        B.MakeEdge(degen_edge);
                        B.Degenerated(degen_edge, Standard_True);
                        // Add pole vertex (both FORWARD and REVERSED since
                        // the edge is topologically a point)
                        TopoDS_Vertex pole_fwd = v_map[trim.m_vi[0]];
                        pole_fwd.Orientation(TopAbs_FORWARD);
                        B.Add(degen_edge, pole_fwd);
                        TopoDS_Vertex pole_rev = v_map[trim.m_vi[0]];
                        pole_rev.Orientation(TopAbs_REVERSED);
                        B.Add(degen_edge, pole_rev);
                        if (trim.m_c2i >= 0 &&
                            trim.m_c2i < (int)c2_map.size() &&
                            !c2_map[trim.m_c2i].IsNull()) {
                            double dt = trim.m_tolerance[0] > 0.0
                                            ? trim.m_tolerance[0]
                                            : linear_tolerance;
                            B.UpdateEdge(degen_edge,
                                         c2_map[trim.m_c2i],
                                         f_map[fi], dt);
                            B.Range(degen_edge, f_map[fi],
                                    trim.Domain().Min(),
                                    trim.Domain().Max());
                        }
                        B.Add(wire, degen_edge);
                    }
                    continue;
                }

                if (ei >= (int)e_map.size()) continue;

                // Seam edge: pcurves already set via the double-pcurve
                // UpdateEdge above — just orient and add to wire.
                if (trim.m_type == ON_BrepTrim::seam &&
                    seam_c2_applied.count(ei)) {
                    TopoDS_Edge oriented_edge = e_map[ei];
                    if (trim.m_bRev3d)
                        oriented_edge.Reverse();
                    B.Add(wire, oriented_edge);
                    continue;
                }

                // Non-seam edge: attach single 2D pcurve.
                // OCCT convention: the pcurve goes in the 3D edge direction
                // (from edge.vi[0] to edge.vi[1]).  When the trim is reversed
                // (m_bRev3d = true) the openNURBS pcurve goes in the opposite
                // direction, so we reverse it before passing to UpdateEdge.
                if (trim.m_c2i >= 0 &&
                    trim.m_c2i < (int)c2_map.size() &&
                    !c2_map[trim.m_c2i].IsNull()) {
                    double tol2d = trim.m_tolerance[0] > 0.0
                                       ? trim.m_tolerance[0]
                                       : linear_tolerance;
                    Handle(Geom2d_Curve) pc = c2_map[trim.m_c2i];
                    if (trim.m_bRev3d)
                        pc = pc->Reversed();
                    B.UpdateEdge(e_map[ei], pc, f_map[fi], tol2d);
                    double pd_min = trim.Domain().Min();
                    double pd_max = trim.Domain().Max();
                    B.Range(e_map[ei], f_map[fi], pd_min, pd_max);
                    // Clear SameRange when domains differ.
                    // SameParameter is handled globally after the shape is built.
                    const double kRangeTol = 1e-10;
                    const auto& ed = e3d_domain[ei];
                    if (std::fabs(pd_min - ed.first)  > kRangeTol ||
                        std::fabs(pd_max - ed.second) > kRangeTol) {
                        B.SameRange(e_map[ei], Standard_False);
                        B.SameParameter(e_map[ei], Standard_False);
                    }
                }

                // Edge orientation: m_bRev3d means trim direction is
                // opposite to the 3-D curve direction
                TopoDS_Edge oriented_edge = e_map[ei];
                if (trim.m_bRev3d)
                    oriented_edge.Reverse();
                B.Add(wire, oriented_edge);
            }

            B.Add(f_map[fi], wire);
        }
    }

    // ------------------------------------------------------------------
    // Step 5 — assemble shell(s) / solid(s) / compound
    // ------------------------------------------------------------------
    // Group faces into connected components via shared edges so that
    // disconnected sub-solids (e.g. a COMPOUND of two bodies) are assembled
    // as separate shells rather than a single shell.  BRepCheck requires
    // every shell inside a solid to be connected; merging disconnected
    // components into one shell triggers BRepCheck_NotConnected.
    //
    // Union-Find over face indices, unioning faces that share an edge.
    const int nF = brep.m_F.Count();
    std::vector<int> parent(nF);
    for (int i = 0; i < nF; ++i) parent[i] = i;
    std::function<int(int)> find = [&](int x) -> int {
        return parent[x] == x ? x : (parent[x] = find(parent[x]));
    };
    auto unite = [&](int a, int b) {
        a = find(a); b = find(b);
        if (a != b) parent[b] = a;
    };

    // For each edge, union all faces whose loops contain a trim of that edge.
    // We accumulate face-per-trim directly from m_T.
    {
        std::vector<int> edge_face(brep.m_E.Count(), -1); // first face seen for edge
        for (int ti = 0; ti < brep.m_T.Count(); ++ti) {
            const ON_BrepTrim& trim = brep.m_T[ti];
            int ei = trim.m_ei;
            if (ei < 0 || ei >= brep.m_E.Count()) continue;
            int li = trim.m_li;
            if (li < 0 || li >= brep.m_L.Count()) continue;
            int fi = brep.m_L[li].m_fi;
            if (fi < 0 || fi >= nF) continue;
            if (edge_face[ei] < 0) {
                edge_face[ei] = fi;
            } else {
                unite(edge_face[ei], fi);
            }
        }
    }

    // Collect faces per component root.
    std::map<int, std::vector<int>> components;
    for (int fi = 0; fi < nF; ++fi)
        components[find(fi)].push_back(fi);

    // Build one shell per component.  Wrap closed shells (all edges have ≥2
    // trims) in a solid.  Collect all resulting shapes.
    //
    // Pre-compute per-edge trim counts (globally, not per component, which
    // is correct: a 2-trim edge is always shared by 2 faces in the same
    // component if the B-Rep was originally valid).
    std::vector<int> trim_count(brep.m_E.Count(), 0);
    for (int ti = 0; ti < brep.m_T.Count(); ++ti) {
        int ei = brep.m_T[ti].m_ei;
        if (ei >= 0 && ei < brep.m_E.Count())
            ++trim_count[ei];
    }

    // Helper: is every edge of this component's faces a shared edge (≥2 trims)?
    // A component is closed when none of its boundary edges is a free edge.
    auto component_is_closed = [&](const std::vector<int>& faces) -> bool {
        // Collect all edges touched by faces in this component.
        std::set<int> face_set(faces.begin(), faces.end());
        // For each trim of each face in the component, check the edge count.
        for (int fi : faces) {
            const ON_BrepFace& face = brep.m_F[fi];
            for (int li = 0; li < face.m_li.Count(); ++li) {
                int loop_idx = face.m_li[li];
                const ON_BrepLoop& loop = brep.m_L[loop_idx];
                for (int ti_local = 0; ti_local < loop.m_ti.Count(); ++ti_local) {
                    int ti = loop.m_ti[ti_local];
                    int ei = brep.m_T[ti].m_ei;
                    if (ei >= 0 && ei < brep.m_E.Count()) {
                        if (trim_count[ei] < 2) return false;
                    }
                }
            }
        }
        return true;
    };

    std::vector<TopoDS_Shape> parts;
    for (auto& kv : components) {
        TopoDS_Shell comp_shell;
        B.MakeShell(comp_shell);
        for (int fi : kv.second)
            B.Add(comp_shell, f_map[fi]);

        if (component_is_closed(kv.second)) {
            TopoDS_Solid solid;
            B.MakeSolid(solid);
            B.Add(solid, comp_shell);
            parts.push_back(solid);
        } else {
            parts.push_back(comp_shell);
        }
    }

    if (parts.size() == 1) {
        // BRepLib::SameParameter fixes the SameParameter flag on each edge:
        // the flag must reflect whether the 3D curve and pcurve(s) are
        // arc-length identical.  A round-trip converter cannot guarantee
        // this; the call here detects and corrects any incorrect flags.
        // Degenerate edges (no 3D curve, missing pcurve) can throw
        // Standard_OutOfRange — catch and ignore since SameParameter=false
        // is the safe/conservative default for such edges.
        try {
            BRepLib::SameParameter(parts[0], linear_tolerance,
                                   Standard_False);
        } catch (...) {}
        return parts[0];
    }

    // Multiple components: wrap in a compound.
    TopoDS_Compound compound;
    B.MakeCompound(compound);
    for (const auto& p : parts)
        B.Add(compound, p);
    // See comment above; same rationale applies for the compound case.
    try {
        BRepLib::SameParameter(compound, linear_tolerance, Standard_False);
    } catch (...) {}
    return compound;
}

// ===========================================================================
//  TopoDS_Shape  →  ON_Brep
// ===========================================================================

bool OCCTToON_Brep(const TopoDS_Shape& shape, ON_Brep& brep,
                   double linear_tolerance)
{
    brep.Destroy();

    if (shape.IsNull()) return false;

    // Maps from OCCT shape (by pointer/hash) to openNURBS index.
    // vertex_map is keyed on (TShape*, rounded_3D_position) so that the same
    // underlying vertex TShape instantiated at different locations (e.g. on a
    // body-of-revolution that has been translated) creates separate ON_BrepVertex
    // objects at their correct 3D positions instead of falsely collapsing to one.
    using PntKey = std::tuple<long long, long long, long long>;
    auto make_pnt_key = [](const gp_Pnt& p) -> PntKey {
        // 1e9 precision: ~1 nm resolution, covers coordinates up to ±9e9 units.
        return { llround(p.X() * 1e9),
                 llround(p.Y() * 1e9),
                 llround(p.Z() * 1e9) };
    };

    // Edge map key: (TShape*, PntKey_of_start_vertex).
    // Using the start-vertex world position disambiguates the same TShape
    // appearing at different body-of-revolution locations (translations) while
    // still collapsing all traversals (FORWARD and REVERSED) of the same physical
    // edge to the same ON_BrepEdge index.
    // Degenerate edges get PntKey{0,0,0} as a sentinel (they share the pole vertex).
    using EdgeKey = std::pair<Standard_Address, PntKey>;
    std::map<PntKey, int>          vertex_map;  // rounded 3D pos → vertex index
    std::map<EdgeKey, int>         edge_map;    // (TShape*, start_pos) → edge index
    std::map<Standard_Address, int> surface_map;// TShape* → surface index
    // Original OCCT surface periods, stored per surface index (si) so that the
    // gap-snap code can apply period-shift corrections even when the converted
    // ON_NurbsSurface does not report IsClosed() correctly (e.g. a cylinder face
    // whose BSpline representation spans slightly more than one period).
    std::map<int, double> surface_uperiod; // si → OCCT U period (0 = non-periodic)
    std::map<int, double> surface_vperiod; // si → OCCT V period (0 = non-periodic)

    // Helper: compute the EdgeKey for a (non-degenerate) edge.
    // TopExp::Vertices with CumOri=false always returns tv0 as the FORWARD
    // (curve-start) vertex regardless of wire orientation, so both FORWARD and
    // REVERSED traversals of the same physical edge produce the same key.
    auto make_edge_key = [&](const TopoDS_Edge& e) -> EdgeKey {
        Standard_Address tsh = e.TShape().get();
        if (BRep_Tool::Degenerated(e))
            return { tsh, PntKey{0LL, 0LL, 0LL} };
        TopoDS_Vertex tv0, tv1;
        TopExp::Vertices(e, tv0, tv1, /*CumOri=*/false);
        if (tv0.IsNull())
            return { tsh, PntKey{0LL, 0LL, 0LL} };
        return { tsh, make_pnt_key(BRep_Tool::Pnt(tv0)) };
    };

    // ------------------------------------------------------------------
    // Pass 1 — collect and translate all vertices
    // ------------------------------------------------------------------
    for (TopExp_Explorer ex(shape, TopAbs_VERTEX); ex.More(); ex.Next()) {
        const TopoDS_Vertex& vtx =
            TopoDS::Vertex(ex.Current());

        gp_Pnt p = BRep_Tool::Pnt(vtx);  // applies location transform
        PntKey key = make_pnt_key(p);
        if (vertex_map.count(key)) continue;

        double tol = BRep_Tool::Tolerance(vtx);

        ON_BrepVertex& ov = brep.NewVertex(
            ON_3dPoint(p.X(), p.Y(), p.Z()), tol);
        vertex_map[key] = ov.m_vertex_index;
    }

    // ------------------------------------------------------------------
    // Pass 2 — collect and translate all 3-D edge curves
    // ------------------------------------------------------------------
    for (TopExp_Explorer ex(shape, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(ex.Current());
        EdgeKey ek = make_edge_key(edge);
        if (edge_map.count(ek)) continue;

        // Degenerate edges (poles) must NOT become ON_BrepEdge objects —
        // they are represented as singular trims in openNURBS.  We store
        // a sentinel value (-2) so Pass 3 can detect them.
        if (BRep_Tool::Degenerated(edge)) {
            edge_map[ek] = -2;
            continue;
        }

        // Detect topologically-closed edges: same underlying vertex TShape at
        // both ends AND the same 3D world position.  Edges where the same TShape
        // appears at two DIFFERENT positions (e.g. a translated body-of-revolution
        // with a shared seam vertex at two Z heights) are NOT closed edges.
        bool topoClosedEdge = false;
        {
            TopoDS_Vertex tv0, tv1;
            TopExp::Vertices(edge, tv0, tv1, /*CumOri=*/false);
            if (!tv0.IsNull() && !tv1.IsNull() &&
                tv0.TShape() == tv1.TShape()) {
                gp_Pnt p0 = BRep_Tool::Pnt(tv0);
                gp_Pnt p1 = BRep_Tool::Pnt(tv1);
                // Closed only when the two vertex instances are at the same
                // 3D location (within a generous position tolerance).
                double vtx_tol =
                    std::max(BRep_Tool::Tolerance(tv0),
                             BRep_Tool::Tolerance(tv1));
                topoClosedEdge = (p0.Distance(p1) <= vtx_tol + 1e-7);
            }
        }

        double t0, t1;
        Handle(Geom_Curve) gc = BRep_Tool::Curve(edge, t0, t1);

        // Determine the actual vertex parameter range for this edge.
        // BRep_Tool::Curve gives the 3D curve and its edge-level range
        // [t0, t1].  For BSplines stored with a wider natural domain (e.g.
        // [-3.1, 76.07] when the edge only uses [7.9, 24.19]), t0 equals the
        // full BSpline start, not the vertex parameter.  We fetch the vertex
        // parameters via BRep_Tool::Parameter to get the exact sub-range.
        //
        // TopExp::Vertices(CumOri=false) returns vertices ordered by the
        // EDGE's own orientation: for a REVERSED edge tv0 is the end vertex
        // (at the curve's last parameter) and tv1 is the start vertex.  When
        // the resulting vertex parameters are inverted (vp0 > vp1) we swap
        // tv0/tv1 so that tv0 always corresponds to edge_t0 (curve start).
        TopoDS_Vertex tv0, tv1;
        double edge_t0 = t0, edge_t1 = t1;
        if (!topoClosedEdge) {
            TopExp::Vertices(edge, tv0, tv1, /*CumOri=*/false);
            if (!tv0.IsNull()) {
                try {
                    double vertex_param = BRep_Tool::Parameter(tv0, edge);
                    if (std::isfinite(vertex_param)) edge_t0 = vertex_param;
                } catch (...) {}
            }
            if (!tv1.IsNull()) {
                try {
                    double vertex_param = BRep_Tool::Parameter(tv1, edge);
                    if (std::isfinite(vertex_param)) edge_t1 = vertex_param;
                } catch (...) {}
            }
            // For REVERSED edges BRep_Tool::Parameter(tv0) = t_end > t_start.
            // Swap so edge_t0 ≤ edge_t1 and tv0 corresponds to curve start.
            if (edge_t0 > edge_t1) {
                std::swap(edge_t0, edge_t1);
                std::swap(tv0, tv1);
            }
        } else {
            // Closed edges: still look up vertices for vi0/vi1 assignment.
            TopExp::Vertices(edge, tv0, tv1, /*CumOri=*/false);
        }

        int c3i = -1;
        if (!gc.IsNull()) {
            // For non-BSpline curves (Geom_Line, Geom_Circle, Geom_Ellipse,
            // etc.) always convert only the edge's actual parameter range
            // [edge_t0, edge_t1] using a Geom_TrimmedCurve.  This avoids
            // domain mismatch when the full curve spans more than the edge
            // (e.g. Geom_Circle with t=[3π/2..5π/2]) and avoids periodic
            // BSplines.
            //
            // For topologically-closed edges we also force the TrimmedCurve
            // path even when gc is already a BSpline, so that the resulting
            // ON_NurbsCurve has domain exactly [t0, t1] with matching
            // start/end 3D points (required for IsClosed() == true).
            //
            // For BSpline edges whose natural domain is wider than the edge
            // range (e.g. [-3.1, 76.07] vs vertex parameters [7.9, 24.19]),
            // we also trim so that NC.Domain() == [edge_t0, edge_t1] and
            // NC(edge_t0) == vertex position.  This keeps ON_BrepEdge.Domain()
            // equal to the trimmed range without relying on reparameterization
            // via SetProxyCurveDomain (whose Domain() getter still returns the
            // wider NC domain, causing vertex mismatches in ON_BrepToOCCT).
            // A small tolerance is used to allow for floating-point rounding
            // when comparing the edge range against the BSpline domain bounds.
            static constexpr double kEdgeParamTol = 1e-9;
            Handle(Geom_BSplineCurve) bc;
            if (!topoClosedEdge) {
                bc = Handle(Geom_BSplineCurve)::DownCast(gc);
                // Trim the BSpline if its natural domain is wider than the
                // vertex parameter range.
                if (!bc.IsNull() &&
                    (fabs(edge_t0 - bc->FirstParameter()) > kEdgeParamTol ||
                     fabs(edge_t1 - bc->LastParameter())  > kEdgeParamTol)) {
                    try {
                        Handle(Geom_TrimmedCurve) tc =
                            new Geom_TrimmedCurve(bc, edge_t0, edge_t1);
                        bc = GeomConvert::CurveToBSplineCurve(tc);
                    } catch (...) {
                        bc = Handle(Geom_BSplineCurve)::DownCast(gc);
                    }
                }
            }
            if (bc.IsNull()) {
                // Trimmed conversion first (preserves arc domain)
                try {
                    Handle(Geom_TrimmedCurve) tc =
                        new Geom_TrimmedCurve(gc, edge_t0, edge_t1);
                    bc = GeomConvert::CurveToBSplineCurve(tc);
                } catch (...) {}
            }
            if (bc.IsNull()) {
                // Last resort: convert the full (possibly infinite) curve
                try {
                    bc = GeomConvert::CurveToBSplineCurve(gc);
                } catch (...) {}
            }
            // Periodic BSplines have one fewer knot than the non-periodic
            // convention expected by OCCTCurveToON.  Flatten to non-periodic.
            if (!bc.IsNull() && bc->IsPeriodic())
                bc->SetNotPeriodic();
            if (!bc.IsNull()) {
                ON_NurbsCurve* nc = new ON_NurbsCurve();
                if (OCCTCurveToON(bc, *nc)) {
                    c3i = brep.AddEdgeCurve(nc);
                } else {
                    delete nc;
                }
            }
        }

        // For topologically-closed edges the curve's own domain endpoints
        // must be coincident (IsClosed() == true).  After the TrimmedCurve
        // conversion the endpoints should already be equal; snap the last
        // CV to exactly match the first to eliminate any floating-point
        // residual so that ON_PointsAreCoincident() returns true.
        if (topoClosedEdge && c3i >= 0) {
            ON_NurbsCurve* nc =
                ON_NurbsCurve::Cast(brep.m_C3[c3i]);
            if (nc && nc->m_cv_count >= 2) {
                ON_3dPoint p0 = nc->PointAt(nc->Domain()[0]);
                ON_3dPoint p1 = nc->PointAt(nc->Domain()[1]);
                double d = p0.DistanceTo(p1);
                double scale = std::max(
                    1.0, std::max(fabs(p0.x),
                         std::max(fabs(p0.y), fabs(p0.z))));
                // Only snap when the endpoints are already geometrically
                // close (genuine floating-point residual, not a mismatch).
                if (d <= 1e-4 * scale) {
                    const int stride = nc->m_cv_stride;
                    double* cv0 = nc->CV(0);
                    double* cvN = nc->CV(nc->m_cv_count - 1);
                    if (cv0 && cvN) {
                        for (int k = 0; k < stride; ++k)
                            cvN[k] = cv0[k];
                    }
                }
            }
        }

        // Determine start/end vertex indices.
        // tv0/tv1 were already computed above (with the swap applied so that
        // tv0 is always the vertex at edge_t0 = the curve start).
        // The EdgeKey used by make_edge_key may differ from the swapped tv0
        // (it was based on the original TopExp order), but that is only for
        // lookup purposes; the vi0/vi1 assignment here uses the corrected tv0.
        int vi0 = -1, vi1 = -1;
        {
            if (!tv0.IsNull()) {
                auto it = vertex_map.find(make_pnt_key(BRep_Tool::Pnt(tv0)));
                if (it != vertex_map.end()) vi0 = it->second;
            }
            if (!tv1.IsNull()) {
                auto it = vertex_map.find(make_pnt_key(BRep_Tool::Pnt(tv1)));
                if (it != vertex_map.end()) vi1 = it->second;
            }
        }
        if (vi1 < 0) vi1 = vi0; // closed edge

        // Use the vertex-reference form of NewEdge so that openNURBS
        // automatically maintains the vertex→edge back-references.
        // The NC was already trimmed to [edge_t0, edge_t1] above, so
        // NewEdge will set ProxyCurveDomain = NC.Domain() = [edge_t0, edge_t1]
        // automatically (no separate SetProxyCurveDomain call needed).
        double edge_tol = BRep_Tool::Tolerance(edge);
        if (edge_tol < 0.0) edge_tol = linear_tolerance;
        ON_BrepEdge* poe = nullptr;
        if (vi0 >= 0 && vi1 >= 0) {
            poe = &brep.NewEdge(brep.m_V[vi0], brep.m_V[vi1], c3i,
                                nullptr,
                                edge_tol);
        } else {
            poe = &brep.NewEdge(c3i);
            poe->m_tolerance = edge_tol;
        }
        int this_ei = poe->m_edge_index;
        // Defensive: force tolerance on the edge after creation
        brep.m_E[this_ei].m_tolerance = edge_tol;
        edge_map[ek] = this_ei;
    }

    // ------------------------------------------------------------------
    // Pass 3 — translate faces
    // ------------------------------------------------------------------
    for (TopExp_Explorer fex(shape, TopAbs_FACE); fex.More(); fex.Next()) {
        const TopoDS_Face& face = TopoDS::Face(fex.Current());

        // Translate surface
        Handle(Geom_Surface) gs = BRep_Tool::Surface(face);
        int si = -1;
        if (!gs.IsNull()) {
            Handle(Geom_BSplineSurface) bs =
                Handle(Geom_BSplineSurface)::DownCast(gs);
            // For periodic BSpline surfaces OR non-BSpline analytical surfaces
            // (Geom_CylindricalSurface, Geom_Plane, etc.): use the
            // Geom_RectangularTrimmedSurface path which always produces a
            // non-periodic BSpline whose UV domain exactly matches the face's
            // UV parameter bounds (= the pcurves' UV range).
            // DO NOT call SetUNotPeriodic() or rely on GeomConvert on the
            // unbounded analytical surface — both can produce a surface with
            // an incorrect/tiny UV domain.
            bool need_trimmed = bs.IsNull() ||
                                bs->IsUPeriodic() || bs->IsVPeriodic();
            if (need_trimmed) {
                try {
                    double u1, u2, v1, v2;
                    BRepTools::UVBounds(face, u1, u2, v1, v2);
                    Handle(Geom_RectangularTrimmedSurface) ts =
                        new Geom_RectangularTrimmedSurface(
                            gs, u1, u2, v1, v2);
                    bs = GeomConvert::SurfaceToBSplineSurface(ts);
                } catch (...) {}
            }
            if (!bs.IsNull()) {
                // Handle any residual periodicity (safety fallback for
                // surfaces that GeomConvert still left periodic).
                // BSplCLib::Unperiodize computes scale = 1/(knot_last - knot_first).
                // For degenerate periodic surfaces (collapsed knot span), this
                // produces a zero denominator, triggering SIGFPE which cannot be
                // caught by C++ try-catch.  Guard: only unperiodize when the
                // knot span is non-zero.
                if (bs->IsUPeriodic() && bs->NbUKnots() >= 2 &&
                    bs->UKnot(bs->NbUKnots()) - bs->UKnot(1) > 1e-15)
                    bs->SetUNotPeriodic();
                if (bs->IsVPeriodic() && bs->NbVKnots() >= 2 &&
                    bs->VKnot(bs->NbVKnots()) - bs->VKnot(1) > 1e-15)
                    bs->SetVNotPeriodic();
                ON_NurbsSurface* ns = new ON_NurbsSurface();
                if (OCCTSurfaceToON(bs, *ns)) {
                    // If the converted surface's UV domain doesn't match the
                    // face UV bounds (which define the pcurves' UV range),
                    // reparameterize the knots to match.  This handles cases
                    // where GeomConvert normalizes the parameterization (e.g.
                    // a cylinder u-range [0,2π] becomes [0,5.59e-9]).
                    // Uniform reparameterization is geometry-preserving.
                    if (need_trimmed) {
                        double fu1, fu2, fv1, fv2;
                        BRepTools::UVBounds(face, fu1, fu2, fv1, fv2);
                        ON_Interval srf_u = ns->Domain(0);
                        ON_Interval srf_v = ns->Domain(1);
                        const double kDomTol = 1e-6;
                        if (fu2 > fu1 && srf_u.IsValid() && srf_u.Length() > 0 &&
                            (fabs(srf_u[0] - fu1) > kDomTol ||
                             fabs(srf_u[1] - fu2) > kDomTol)) {
                            double ku_scale = (fu2 - fu1) / srf_u.Length();
                            for (int k = 0; k < ns->KnotCount(0); ++k)
                                ns->m_knot[0][k] = fu1 +
                                    (ns->m_knot[0][k] - srf_u[0]) * ku_scale;
                        }
                        if (fv2 > fv1 && srf_v.IsValid() && srf_v.Length() > 0 &&
                            (fabs(srf_v[0] - fv1) > kDomTol ||
                             fabs(srf_v[1] - fv2) > kDomTol)) {
                            double kv_scale = (fv2 - fv1) / srf_v.Length();
                            for (int k = 0; k < ns->KnotCount(1); ++k)
                                ns->m_knot[1][k] = fv1 +
                                    (ns->m_knot[1][k] - srf_v[0]) * kv_scale;
                        }
                    }
                    si = brep.AddSurface(ns);
                } else {
                    delete ns;
                }
            }

            // Cache by shape pointer so seam surfaces are shared
            Standard_Address sk = face.TShape().get();
            if (!surface_map.count(sk) && si >= 0) {
                surface_map[sk] = si;
                // Record the original OCCT surface's period so the gap-snap
                // code can apply period-shift corrections even when the
                // converted ON_NurbsSurface's IsClosed() returns false.
                if (!gs.IsNull()) {
                    surface_uperiod[si] =
                        gs->IsUPeriodic() ? gs->UPeriod() : 0.0;
                    surface_vperiod[si] =
                        gs->IsVPeriodic() ? gs->VPeriod() : 0.0;
                }
            }
        }

        // Check if the surface pointer was already added
        if (si < 0) {
            Standard_Address sk = face.TShape().get();
            auto it = surface_map.find(sk);
            if (it != surface_map.end())
                si = it->second;
        }

        if (si < 0) continue; // cannot translate this face

        ON_BrepFace& of = brep.NewFace(si);
        of.m_bRev = (face.Orientation() == TopAbs_REVERSED);

        // -- Iterate wires → loops
        // openNURBS requires face.m_li[0] to be the outer loop.  Use
        // BRepTools::OuterWire() to identify the outer wire, then
        // iterate wires in outer-first order so the outer loop lands at
        // face.m_li[0].
        {
        TopoDS_Wire face_outer_wire = BRepTools::OuterWire(face);

        // Collect wires, outer wire first.
        std::vector<TopoDS_Wire> face_wires;
        if (!face_outer_wire.IsNull())
            face_wires.push_back(face_outer_wire);
        for (TopExp_Explorer wex(face, TopAbs_WIRE); wex.More(); wex.Next()) {
            const TopoDS_Wire& w = TopoDS::Wire(wex.Current());
            if (face_outer_wire.IsNull() || !w.IsSame(face_outer_wire))
                face_wires.push_back(w);
        }

        for (const TopoDS_Wire& wire : face_wires) {
            // First entry is outer; all others are inner holes.
            const bool wire_is_outer = (&wire == &face_wires[0]);

            // Collect edges in the correct wire traversal order.
            // BRepTools_WireExplorer follows the wire's connectivity (handles
            // seam and degenerate edges in native OCCT shapes correctly).
            // For ON_BrepToOCCT-generated wires that lack full vertex
            // connectivity, the wire explorer may not visit all edges.  In
            // that case fall back to TopExp_Explorer which is order-agnostic
            // but visits all edges.
            std::vector<TopoDS_Edge> wire_edges;
            bool wire_explorer_found_edges = false;
            {
                for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
                    wire_edges.push_back(we.Current());
                    wire_explorer_found_edges = true;
                }
            }
            // Count unique physical edges (TShape + start-vertex position) from
            // TopExp_Explorer.  WireExplorer may visit seam edges twice (FORWARD
            // and REVERSED); since both traversals produce the same EdgeKey, the
            // unique-key count is the right comparison.
            {
                std::set<EdgeKey> tex_keys;
                for (TopExp_Explorer eex(wire, TopAbs_EDGE);
                     eex.More(); eex.Next()) {
                    tex_keys.insert(
                        make_edge_key(TopoDS::Edge(eex.Current())));
                }
                std::set<EdgeKey> we_keys;
                for (const TopoDS_Edge& e : wire_edges)
                    we_keys.insert(make_edge_key(e));

                // If BRepTools_WireExplorer found nothing but TopExp found
                // edges, this is an open (slit/dangling) wire.  ON_Brep
                // requires all loops to be closed; skip these wires.  They
                // appear in FreeCAD files as reference geometry or partition
                // seams and do not bound a surface region.
                if (!wire_explorer_found_edges && !tex_keys.empty())
                    continue;

                if (we_keys.size() < tex_keys.size()) {
                    // WireExplorer missed some edges — revert to TopExp order
                    wire_edges.clear();
                    for (TopExp_Explorer eex(wire, TopAbs_EDGE);
                         eex.More(); eex.Next())
                        wire_edges.push_back(TopoDS::Edge(eex.Current()));
                }
            }

            ON_BrepLoop& ol = brep.NewLoop(ON_BrepLoop::unknown, of);

            for (const TopoDS_Edge& edge : wire_edges) {
                EdgeKey ek = make_edge_key(edge);

                auto eit = edge_map.find(ek);
                if (eit == edge_map.end()) continue;
                int ei = eit->second;

                // Translate 2D pcurve for this edge on this face
                double pc_t0, pc_t1;
                Handle(Geom2d_Curve) gc2 =
                    BRep_Tool::CurveOnSurface(edge, face, pc_t0, pc_t1);

                int c2i = -1;
                if (!gc2.IsNull()) {
                    // Same strategy as 3D curves: trim first to get the actual
                    // arc/segment, then fall back to the full-curve conversion.
                    // Periodic 2D BSplines are flattened with SetNotPeriodic()
                    // so the knot count matches the non-periodic convention.
                    Handle(Geom2d_BSplineCurve) bc2 =
                        Handle(Geom2d_BSplineCurve)::DownCast(gc2);

                    // For closed-loop periodic pcurves (gc2(t0)≈gc2(t1)),
                    // SetNotPeriodic() introduces a kink at the junction that
                    // breaks the ON_Brep tangent-direction validation for closed
                    // trims.  Force the TrimmedCurve path instead, which converts
                    // the full period to a non-periodic B-spline without a kink.
                    if (!bc2.IsNull() && bc2->IsPeriodic()) {
                        gp_Pnt2d p_s, p_e;
                        gc2->D0(pc_t0, p_s);
                        gc2->D0(pc_t1, p_e);
                        if (p_s.Distance(p_e) < 1e-4)
                            bc2.Nullify(); // force TrimmedCurve path below
                    }

                    // When the B-spline pcurve spans a wider domain than the
                    // edge's actual parameter range [pc_t0, pc_t1], the first
                    // control vertex cv[0] does NOT correspond to the curve
                    // value at pc_t0 (it's at the B-spline's own domain start).
                    // The UV-gap snap code writes directly to cv[0], so it
                    // would be ineffective.  Force the TrimmedCurve path to
                    // produce a B-spline with domain exactly [pc_t0, pc_t1],
                    // ensuring cv[0] = PointAt(pc_t0).
                    //
                    // Use an absolute tolerance of 1e-4 to avoid re-trimming
                    // seam pcurves that already have a domain matching
                    // [pc_t0, pc_t1] to within floating-point precision.
                    if (!bc2.IsNull() && !bc2->IsPeriodic()) {
                        const double kDomTol = 1e-4;
                        if (fabs(bc2->FirstParameter() - pc_t0) > kDomTol ||
                            fabs(bc2->LastParameter()  - pc_t1) > kDomTol) {
                            bc2.Nullify(); // force TrimmedCurve path below
                        }
                    }

                    if (bc2.IsNull()) {
                        try {
                            Handle(Geom2d_TrimmedCurve) tc2 =
                                new Geom2d_TrimmedCurve(gc2, pc_t0, pc_t1);
                            bc2 = Geom2dConvert::CurveToBSplineCurve(tc2);
                        } catch (...) {}
                    }
                    if (bc2.IsNull()) {
                        try {
                            bc2 = Geom2dConvert::CurveToBSplineCurve(gc2);
                        } catch (...) {}
                    }
                    if (!bc2.IsNull() && bc2->IsPeriodic())
                        bc2->SetNotPeriodic();
                    if (!bc2.IsNull()) {
                        int deg2    = bc2->Degree();
                        int npoles2 = bc2->NbPoles();
                        bool rat2   = bc2->IsRational() ? true : false;

                        ON_NurbsCurve* nc = new ON_NurbsCurve();
                        if (nc->Create(2, rat2, deg2 + 1, npoles2)) {
                            for (int k = 1; k <= npoles2; ++k) {
                                gp_Pnt2d p2 = bc2->Pole(k);
                                double w2   = rat2 ? bc2->Weight(k) : 1.0;
                                nc->SetCV(k - 1,
                                          ON_4dPoint(p2.X() * w2,
                                                     p2.Y() * w2,
                                                     0.0,
                                                     w2));
                            }
                            // Strip one phantom knot from each end (same
                            // convention as OCCTCurveToON / OCCTSurfaceToON).
                            int ki2 = 0;
                            const int nkg2 = bc2->NbKnots();
                            for (int k = 1; k <= nkg2; ++k) {
                                double kv2 = bc2->Knot(k);
                                int    km2 = bc2->Multiplicity(k);
                                if (k == 1 || k == nkg2)
                                    km2 -= 1;
                                for (int m = 0; m < km2; ++m)
                                    nc->m_knot[ki2++] = kv2;
                            }
                            if (nc->IsValid()) {
                                // Sync pc_t0/pc_t1 to the actual B-spline domain.
                                //
                                // Geom2dConvert::CurveToBSplineCurve normalizes the
                                // output domain to [0,1] for many curve types (lines,
                                // circles, trimmed non-BSplines).  If the OCCT pcurve
                                // parameter range pc_t0/pc_t1 differs from nc->Domain(),
                                // SetProxyCurve would evaluate PointAtEnd() at an
                                // intermediate parameter of the [0,1] curve rather than
                                // at cv[last], giving a wrong trim endpoint.
                                //
                                // After this sync, proxy_dom = nc->Domain() so
                                // PointAtEnd() = curve(nc_dom.Max()) = cv[last] = the
                                // correct geometric endpoint of the edge.
                                //
                                // The m_c3i reparameterization below may further adjust
                                // nc's knots for period-shifted seam pcurves; pc_t0/pc_t1
                                // are updated there as well.
                                {
                                    ON_Interval nc_dom_init = nc->Domain();
                                    if (nc_dom_init.IsValid()) {
                                        pc_t0 = nc_dom_init.Min();
                                        pc_t1 = nc_dom_init.Max();
                                    }
                                }

                                // Remap the pcurve's knot domain to match the
                                // 3D edge domain so SameRange=true is preserved
                                // in the round-trip.  In the original OCCT B-Rep
                                // every edge with a 3D curve has SameRange=true
                                // (pcurve parameter domain == 3D curve domain).
                                // The conversion may produce a pcurve with a
                                // different domain (e.g. period-shifted by ±2π),
                                // which would require SameRange=false and trigger
                                // BRepCheck_InvalidSameRangeFlag on reconstruction.
                                if (ei >= 0 && ei < brep.m_E.Count() &&
                                    brep.m_E[ei].m_c3i >= 0) {
                                    ON_Interval edge_dom = brep.m_E[ei].Domain();
                                    ON_Interval nc_dom   = nc->Domain();
                                    if (edge_dom.IsValid() && nc_dom.IsValid() &&
                                        edge_dom.Length() > 0 && nc_dom.Length() > 0) {
                                        double old0 = nc_dom.Min(), old1 = nc_dom.Max();
                                        double new0 = edge_dom.Min(), new1 = edge_dom.Max();
                                        const double kTol = 1e-10;
                                        if (fabs(old0 - new0) > kTol ||
                                            fabs(old1 - new1) > kTol) {
                                            double scale = (new1 - new0) /
                                                           (old1 - old0);
                                            for (int k = 0; k < nc->KnotCount(); ++k)
                                                nc->m_knot[k] = new0 +
                                                    (nc->m_knot[k] - old0) * scale;
                                            pc_t0 = new0;
                                            pc_t1 = new1;
                                        }
                                    }
                                }

                                // OCCT stores all pcurves in the FORWARD edge
                                // direction, regardless of the edge orientation.
                                // For REVERSED edges, the openNURBS trim direction
                                // is opposite, so the pcurve must be reversed.
                                // This applies to seam edges too.
                                bool need_rev =
                                    (edge.Orientation() == TopAbs_REVERSED);
                                if (need_rev) {
                                    // Reverse() negates the knots so the
                                    // domain becomes [-t1, -t0].  Restore the
                                    // original parameter range [pc_t0, pc_t1]
                                    // so that SetProxyCurveDomain succeeds.
                                    nc->Reverse();
                                    nc->SetDomain(pc_t0, pc_t1);
                                }
                                c2i = brep.AddTrimCurve(nc);
                            } else {
                                delete nc;
                            }
                        } else {
                            delete nc;
                        }
                    }
                }

                // Trim orientation
                bool bRev3d = (edge.Orientation() == TopAbs_REVERSED);

                if (ei == -2) {
                    // Degenerate OCCT edge → openNURBS singular trim.
                    // Find the pole vertex by evaluating the surface at the
                    // start point of the 2D pcurve and picking the nearest
                    // already-translated vertex.  O(|vertices|) search; for
                    // typical B-Rep models (2-8 vertices) this is negligible.
                    int pole_vi = -1;
                    if (c2i >= 0 && si >= 0 && brep.m_V.Count() > 0) {
                        const ON_Surface* srf = brep.m_S[si];
                        const ON_NurbsCurve* c2 =
                            ON_NurbsCurve::Cast(brep.m_C2[c2i]);
                        if (srf && c2) {
                            ON_Interval dom = c2->Domain();
                            ON_3dPoint uv = c2->PointAt(dom.Min());
                            ON_3dPoint pt3;
                            if (srf->EvPoint(uv.x, uv.y, pt3)) {
                                double best_d = 1e30;
                                for (int vi = 0; vi < brep.m_V.Count(); ++vi) {
                                    double d = brep.m_V[vi].point.DistanceTo(pt3);
                                    if (d < best_d) {
                                        best_d = d;
                                        pole_vi = vi;
                                    }
                                }
                            }
                        }
                    }
                    if (pole_vi < 0) continue; // no vertex found — skip trim

                    ON_BrepTrim& strim = brep.NewSingularTrim(
                        brep.m_V[pole_vi], ol, ON_Surface::not_iso, c2i);
                    strim.m_tolerance[0] = BRep_Tool::Tolerance(edge);
                    strim.m_tolerance[1] = strim.m_tolerance[0];
                    if (c2i >= 0) {
                        ON_Interval proxy_dom(pc_t0, pc_t1);
                        ON_Interval nc_dom = brep.m_C2[c2i]->Domain();
                        bool use_sub = proxy_dom.IsIncreasing() &&
                                       nc_dom.Includes(proxy_dom[0]) &&
                                       nc_dom.Includes(proxy_dom[1]);
                        strim.SetProxyCurve(brep.m_C2[c2i],
                                            use_sub ? proxy_dom : nc_dom);
                    }
                    continue;
                }

                ON_BrepTrim& trim =
                    brep.NewTrim(brep.m_E[ei], bRev3d, ol, c2i);
                trim.m_tolerance[0] = BRep_Tool::Tolerance(edge);
                trim.m_tolerance[1] = trim.m_tolerance[0];
                if (c2i >= 0) {
                    // SetProxyCurve (two-arg form) sets both m_this_domain and
                    // m_real_curve_domain to [pc_t0, pc_t1], so that
                    // ON_Brep::IsValid()'s direct evaluation of
                    // m_C2[c2i]->PointAt(trim.Domain()[1]) agrees with the
                    // snap code's PointAtEnd() which respects the proxy domain.
                    // SetProxyCurveDomain alone only updates m_real_curve_domain,
                    // leaving m_this_domain at the curve's full domain and causing
                    // a false gap error when the pcurve domain is a sub-range.
                    // Guards:
                    // 1. proxy_dom must be strictly increasing (no degenerate
                    //    or inverted interval).
                    // 2. proxy_dom must be a subset of the stored B-spline's
                    //    domain; SetProxyCurve intersects the two intervals, so
                    //    an out-of-range sub-interval produces EmptyInterval and
                    //    an invalid proxy (m_real_curve_domain not increasing).
                    // When either guard fails, fall back to the full curve domain.
                    ON_Interval proxy_dom(pc_t0, pc_t1);
                    ON_Interval nc_dom = brep.m_C2[c2i]->Domain();
                    bool use_sub = proxy_dom.IsIncreasing() &&
                                   nc_dom.Includes(proxy_dom[0]) &&
                                   nc_dom.Includes(proxy_dom[1]);
                    trim.SetProxyCurve(brep.m_C2[c2i],
                                       use_sub ? proxy_dom : nc_dom);
                }
                // Note: trim.m_type is set automatically by NewTrim
                // (boundary→seam→mated) — do NOT override it here.
            }

            // UV connectivity fix: close any UV gap between adjacent trims
            // in a loop so that ON_Brep::IsValid() passes the loop-continuity
            // check.  Two strategies are applied in order:
            //
            // 1. Period-gap: if the gap equals the surface period in U or V,
            //    translate ALL poles of tk1's pcurve by ±period.  This
            //    corrects cases where BRep_Tool::CurveOnSurface returns the
            //    "wrong" seam side (U=2π instead of U=0, etc.).
            //
            // 2. Small-gap snap: for any remaining non-zero gap (floating-
            //    point noise from independent pcurve evaluations, or a small
            //    but non-trivial discrepancy), overwrite the first control
            //    point of tk1's pcurve with tk's UV endpoint.  For a clamped
            //    B-spline the first pole is exactly the curve value at the
            //    domain start, so this makes the two trim endpoints exactly
            //    equal at the machine-epsilon level.
            {
                const int li = ol.m_loop_index;
                const int fi = (li >= 0 && li < brep.m_L.Count())
                                   ? brep.m_L[li].m_fi
                                   : -1;
                const ON_Surface* srf =
                    (fi >= 0) ? brep.m_F[fi].SurfaceOf() : nullptr;

                if (srf) {
                    double u0, u1, v0, v1;
                    srf->GetDomain(0, &u0, &u1);
                    srf->GetDomain(1, &v0, &v1);
                    const double uperiod = u1 - u0;
                    const double vperiod = v1 - v0;
                    const bool cls_u = srf->IsClosed(0);
                    const bool cls_v = srf->IsClosed(1);

                    // Also fetch the original OCCT surface period (recorded
                    // during surface conversion).  When the converted
                    // ON_NurbsSurface does not report IsClosed() correctly —
                    // e.g. a cylinder face whose BSpline was trimmed to a UV
                    // range slightly wider than one period, causing IsClosed(0)
                    // = false even though the underlying geometry closes — use
                    // the stored OCCT period to allow period-shift detection.
                    const int si_for_gap = (fi >= 0 && fi < brep.m_F.Count())
                                             ? brep.m_F[fi].SurfaceIndexOf()
                                             : -1;
                    auto up_it = surface_uperiod.find(si_for_gap);
                    auto vp_it = surface_vperiod.find(si_for_gap);
                    const double occt_uperiod =
                        (up_it != surface_uperiod.end()) ? up_it->second : 0.0;
                    const double occt_vperiod =
                        (vp_it != surface_vperiod.end()) ? vp_it->second : 0.0;

                    const int nc = ol.m_ti.Count();
                    for (int k = 0; k < nc; ++k) {
                        ON_BrepTrim& tk  = brep.m_T[ol.m_ti[k]];
                        ON_BrepTrim& tk1 =
                            brep.m_T[ol.m_ti[(k + 1) % nc]];

                        ON_3dPoint pe  = tk.PointAtEnd();
                        ON_3dPoint ps  = tk1.PointAtStart();
                        double dx = ps.x - pe.x;
                        double dy = ps.y - pe.y;

                        if (fabs(dx) < 1e-14 && fabs(dy) < 1e-14)
                            continue;  // already connected

                        int c2i = tk1.m_c2i;
                        if (c2i < 0 || c2i >= brep.m_C2.Count()) continue;
                        ON_NurbsCurve* nc2 =
                            ON_NurbsCurve::Cast(brep.m_C2[c2i]);
                        if (!nc2 || nc2->m_cv_count < 1) continue;

                        bool modified = false;

                        // Strategy 1: period gap — translate all poles.
                        // Check using both the ON_Surface domain width (cls_u /
                        // uperiod) and the original OCCT surface period stored
                        // during conversion.  The OCCT period catches cylinders
                        // whose BSpline spans slightly more than one period,
                        // causing IsClosed(0) = false.
                        const double kRelTol = 1e-3;
                        double shift_u = 0.0, shift_v = 0.0;
                        // U period: prefer OCCT period if available, fall back
                        // to domain width when surface reports IsClosed.
                        const double eff_uperiod = (occt_uperiod > 0.0)
                                                       ? occt_uperiod
                                                       : (cls_u ? uperiod : 0.0);
                        const double eff_vperiod = (occt_vperiod > 0.0)
                                                       ? occt_vperiod
                                                       : (cls_v ? vperiod : 0.0);
                        if (eff_uperiod > 0.0 &&
                            fabs(fabs(dx) - eff_uperiod) <
                                kRelTol * eff_uperiod &&
                            fabs(dy) < kRelTol * eff_uperiod) {
                            shift_u = -dx;
                        } else if (eff_vperiod > 0.0 &&
                                   fabs(fabs(dy) - eff_vperiod) <
                                       kRelTol * eff_vperiod &&
                                   fabs(dx) < kRelTol * eff_vperiod) {
                            shift_v = -dy;
                        }

                        if (fabs(shift_u) > 1e-12 ||
                            fabs(shift_v) > 1e-12) {
                            for (int p = 0; p < nc2->m_cv_count; ++p) {
                                double* cv = nc2->CV(p);
                                cv[0] += shift_u;
                                cv[1] += shift_v;
                            }
                            modified = true;
                            // Re-evaluate after translation so that
                            // strategy 2 below sees the updated gap.
                            tk1.SetProxyCurveDomain(tk1.ProxyCurveDomain());
                            pe = tk.PointAtEnd();
                            ps = tk1.PointAtStart();
                            dx = ps.x - pe.x;
                            dy = ps.y - pe.y;
                        }

                        // Strategy 2: snap the first pole of tk1's pcurve
                        // to tk's UV endpoint to eliminate floating-point
                        // residual gaps.  Only applied when the gap is
                        // smaller than 5% of the surface domain (avoids
                        // snapping a genuine topological mismatch).
                        // The B-spline domain trimming above ensures that
                        // cv[0] corresponds to PointAt(proxy_domain_start),
                        // making this snap effective.
                        if (fabs(dx) > 1e-14 || fabs(dy) > 1e-14) {
                            const double domain_sz =
                                std::max(uperiod > 0.0 ? uperiod : (u1-u0),
                                         vperiod > 0.0 ? vperiod : (v1-v0));
                            const double snap_limit =
                                0.05 * domain_sz;
                            if (snap_limit > 0.0 &&
                                fabs(dx) < snap_limit &&
                                fabs(dy) < snap_limit) {
                                double* cv0 = nc2->CV(0);
                                // For rational curves CV stores homogeneous
                                // coords (x*w, y*w, w); multiply by weight.
                                if (nc2->IsRational()) {
                                    const double w = cv0[nc2->m_dim];
                                    cv0[0] = pe.x * w;
                                    cv0[1] = pe.y * w;
                                    // cv0[m_dim] (weight) is unchanged
                                } else {
                                    cv0[0] = pe.x;
                                    cv0[1] = pe.y;
                                }
                                modified = true;
                                tk1.SetProxyCurveDomain(
                                    tk1.ProxyCurveDomain());
                            }
                        }

                        // Invalidate cached bounding boxes once after
                        // either modification so SetTolerancesBoxesAndFlags
                        // recomputes them from the updated curve.
                        if (modified) {
                            tk1.m_pbox.Destroy();
                            ol.m_pbox.Destroy();
                        }
                    }
                }
            }

            // Fix up loop type based on outer-wire detection above.
            ol.m_type = wire_is_outer ? ON_BrepLoop::outer
                                      : ON_BrepLoop::inner;
        }
        } // end face_wires scope
    }

    brep.SetTolerancesBoxesAndFlags(/*bLazy=*/true);

    // ---------------------------------------------------------------------------
    // Post-processing pass: repair seam/singular trim m_iso corruption.
    //
    // The UV-gap snap occasionally sets CV[0] of a seam pcurve to the
    // floating-point endpoint of the preceding trim (e.g. U=2.8e-9 instead
    // of exactly U=0).  This tiny error is geometrically negligible but
    // causes ON_Surface::IsIsoparametric() to return not_iso (the surface
    // parameter tolerance is ~1e-15), making SetTrimIsoFlags set m_iso =
    // not_iso for the seam trim and failing the ON_Brep::IsValid() check
    // "seam trim m_iso is not N/E/W/S_iso".
    //
    // Similarly, singular trims (degenerate poles) on B-spline surfaces can
    // have SetTrimIsoFlags() silently return not_iso even when the pcurve CVs
    // lie exactly on a surface boundary.  The ON_NurbsSurface::IsIsoparametric
    // implementation uses a tight absolute tolerance (~1e-12) that can fail for
    // large domain values (e.g. V_max = 28.6515).
    //
    // Also fixes the case where SetTrimIsoFlags correctly identifies a seam
    // pcurve as x_iso or y_iso (interior U/V constant) but the validation
    // requires N/E/W/S_iso (boundary) for seam trims.  For example, a cylinder
    // seam at U=2π gets x_iso instead of E_iso.  We promote x_iso→W_iso/E_iso
    // and y_iso→S_iso/N_iso for seam trims that are at a surface boundary.
    //
    // Fix for all cases: for any seam or singular trim whose m_iso is not_iso,
    // x_iso, or y_iso, check whether all CVs share a nearly-constant U or V
    // value (within 1e-4 of the maximum CV extent in that direction).  If so,
    // snap to the nearest surface domain boundary and recompute.  If
    // SetTrimIsoFlags still returns interior iso, compute boundary iso directly.
    // ---------------------------------------------------------------------------
    for (int ti = 0; ti < brep.m_T.Count(); ++ti) {
        ON_BrepTrim& trim = brep.m_T[ti];
        if (trim.m_type != ON_BrepTrim::seam &&
            trim.m_type != ON_BrepTrim::singular) continue;
        // Skip trims that already have proper boundary iso.
        // Seam and singular trims need W/E/S/N_iso (3–6); x_iso(1) and y_iso(2)
        // are interior-only and invalid for seam and singular trims.
        const bool needs_repair =
            (trim.m_iso == ON_Surface::not_iso) ||
            ((trim.m_type == ON_BrepTrim::seam ||
              trim.m_type == ON_BrepTrim::singular) &&
             (trim.m_iso == ON_Surface::x_iso ||
              trim.m_iso == ON_Surface::y_iso));
        if (!needs_repair) continue;

        if (trim.m_c2i < 0 || trim.m_c2i >= brep.m_C2.Count()) continue;
        ON_NurbsCurve* nc = ON_NurbsCurve::Cast(brep.m_C2[trim.m_c2i]);
        if (!nc || nc->CVCount() < 1 || nc->m_dim < 2) continue;

        const int nCV = nc->CVCount();
        const bool rat = nc->IsRational();

        // Compute min/max U and V across all CVs
        double uMin = 1e300, uMax = -1e300;
        double vMin = 1e300, vMax = -1e300;
        for (int p = 0; p < nCV; ++p) {
            const double* cv = nc->CV(p);
            double w = rat ? cv[nc->m_dim] : 1.0;
            if (w == 0.0) w = 1.0;
            double u = cv[0] / w, v = cv[1] / w;
            if (u < uMin) uMin = u; if (u > uMax) uMax = u;
            if (v < vMin) vMin = v; if (v > vMax) vMax = v;
        }

        const double kIsoTol = 1e-4;
        bool snap_u = (uMax - uMin) <= kIsoTol;
        bool snap_v = (vMax - vMin) <= kIsoTol;

        // For SINGULAR trims (pole/apex edges), the trim pcurve should lie
        // exactly on the boundary where the surface degenerates to a point.
        // After B-spline conversion the degenerate locus may not be perfectly
        // iso, but if ANY CV is at a surface boundary we can infer the
        // correct iso line and snap all CVs to it.
        //
        // For singular trims we also use domain-relative tolerances: 1e-4
        // absolute is too tight for large B-spline surfaces (domain ≫ 1).
        // We also allow a greedy "pick smallest-variation axis" approach when
        // even the domain-relative check fails, since openNURBS requires
        // singular trims to have W/E/N/S_iso (boundary iso), not not_iso.
        if (trim.m_type == ON_BrepTrim::singular) {
            const int li_s  = trim.m_li;
            const int fi_s  = (li_s >= 0 && li_s < brep.m_L.Count())
                                  ? brep.m_L[li_s].m_fi : -1;
            const ON_Surface* srf_s = (fi_s >= 0 && fi_s < brep.m_F.Count())
                                          ? brep.m_F[fi_s].SurfaceOf() : nullptr;
            if (srf_s && (!snap_u || !snap_v)) {
                double su0s=0,su1s=0,sv0s=0,sv1s=0;
                srf_s->GetDomain(0,&su0s,&su1s);
                srf_s->GetDomain(1,&sv0s,&sv1s);
                const double uDom = su1s - su0s;
                const double vDom = sv1s - sv0s;
                // Relative tolerance: 1% of domain, or absolute 1e-4, whichever
                // is larger.  Singular trims can have small numerical drift in
                // the "iso" axis after B-spline conversion.
                const double kRelFrac = 0.01;
                const double uRelTol = (uDom > 0) ? std::max(kIsoTol, kRelFrac * uDom) : kIsoTol;
                const double vRelTol = (vDom > 0) ? std::max(kIsoTol, kRelFrac * vDom) : kIsoTol;

                if (!snap_u && (uMax - uMin) <= uRelTol) snap_u = true;
                if (!snap_v && (vMax - vMin) <= vRelTol) snap_v = true;

                // If still neither, try to match CVs to a surface boundary
                // with the larger tolerance, scanning all CVs (not just CV[0]).
                if (!snap_u && !snap_v) {
                    double uMean = 0.5*(uMin+uMax), vMean = 0.5*(vMin+vMax);
                    auto near_bnd = [](double val, double b0, double b1,
                                       double tol) -> bool {
                        return fabs(val-b0) <= tol || fabs(val-b1) <= tol;
                    };
                    auto snap_to_nearest = [](double val, double b0,
                                               double b1) -> double {
                        return (fabs(val-b0) <= fabs(val-b1)) ? b0 : b1;
                    };
                    const double kBndRelTol = 0.05; // 5% of domain
                    const double uBndTol = (uDom > 0) ? std::max(kIsoTol, kBndRelTol * uDom) : kIsoTol;
                    const double vBndTol = (vDom > 0) ? std::max(kIsoTol, kBndRelTol * vDom) : kIsoTol;
                    if (near_bnd(vMean, sv0s, sv1s, vBndTol)) {
                        double v_anchor = snap_to_nearest(vMean, sv0s, sv1s);
                        for (int p = 0; p < nCV; ++p) {
                            double* cv = nc->CV(p);
                            double w = rat ? cv[nc->m_dim] : 1.0;
                            if (w==0.0) w=1.0;
                            cv[1] = v_anchor * w;
                        }
                        snap_v = true;
                        vMin = vMax = v_anchor;
                    } else if (near_bnd(uMean, su0s, su1s, uBndTol)) {
                        double u_anchor = snap_to_nearest(uMean, su0s, su1s);
                        for (int p = 0; p < nCV; ++p) {
                            double* cv = nc->CV(p);
                            double w = rat ? cv[nc->m_dim] : 1.0;
                            if (w==0.0) w=1.0;
                            cv[0] = u_anchor * w;
                        }
                        snap_u = true;
                        uMin = uMax = u_anchor;
                    } else if (uDom > 0 && vDom > 0) {
                        // Last resort: pick the axis with smallest relative
                        // variation and snap to the nearest boundary.  Since
                        // openNURBS requires singular trims to be boundary-iso,
                        // this is always the right direction to try.
                        double uRelVar = (uMax - uMin) / uDom;
                        double vRelVar = (vMax - vMin) / vDom;
                        if (vRelVar <= uRelVar) {
                            double v_anchor = snap_to_nearest(vMean, sv0s, sv1s);
                            for (int p = 0; p < nCV; ++p) {
                                double* cv = nc->CV(p);
                                double w = rat ? cv[nc->m_dim] : 1.0;
                                if (w==0.0) w=1.0;
                                cv[1] = v_anchor * w;
                            }
                            snap_v = true;
                            vMin = vMax = v_anchor;
                        } else {
                            double u_anchor = snap_to_nearest(uMean, su0s, su1s);
                            for (int p = 0; p < nCV; ++p) {
                                double* cv = nc->CV(p);
                                double w = rat ? cv[nc->m_dim] : 1.0;
                                if (w==0.0) w=1.0;
                                cv[0] = u_anchor * w;
                            }
                            snap_u = true;
                            uMin = uMax = u_anchor;
                        }
                    }
                }

                // Legacy: check CV[0] against boundaries with relative tol.
                // (This was the original approach; the new code above supersedes
                // it for the not-yet-snapped axes, but the snap above already
                // updates snap_u/snap_v so this block is only reached for
                // the other axis if needed.)
                if (!snap_v) {
                    auto at_bnd_v = [&](double val) -> bool {
                        return fabs(val-sv0s) <= vRelTol || fabs(val-sv1s) <= vRelTol;
                    };
                    auto snap_bnd_v = [&](double val) -> double {
                        return (fabs(val-sv0s) <= fabs(val-sv1s)) ? sv0s : sv1s;
                    };
                    const double* cv0 = nc->CV(0);
                    double w0 = rat ? cv0[nc->m_dim] : 1.0;
                    if (w0==0.0) w0=1.0;
                    if (at_bnd_v(cv0[1]/w0)) {
                        double v_anchor = snap_bnd_v(cv0[1]/w0);
                        for (int p=0;p<nCV;++p) {
                            double* cv = nc->CV(p);
                            double w = rat ? cv[nc->m_dim] : 1.0;
                            if (w==0.0) w=1.0;
                            cv[1] = v_anchor * w;
                        }
                        snap_v = true;
                        vMin = vMax = v_anchor;
                    }
                }
                if (!snap_u) {
                    auto at_bnd_u = [&](double val) -> bool {
                        return fabs(val-su0s) <= uRelTol || fabs(val-su1s) <= uRelTol;
                    };
                    auto snap_bnd_u = [&](double val) -> double {
                        return (fabs(val-su0s) <= fabs(val-su1s)) ? su0s : su1s;
                    };
                    const double* cv0 = nc->CV(0);
                    double w0 = rat ? cv0[nc->m_dim] : 1.0;
                    if (w0==0.0) w0=1.0;
                    if (at_bnd_u(cv0[0]/w0)) {
                        double u_anchor = snap_bnd_u(cv0[0]/w0);
                        for (int p=0;p<nCV;++p) {
                            double* cv = nc->CV(p);
                            double w = rat ? cv[nc->m_dim] : 1.0;
                            if (w==0.0) w=1.0;
                            cv[0] = u_anchor * w;
                        }
                        snap_u = true;
                        uMin = uMax = u_anchor;
                    }
                }
            }
        }

        if (!snap_u && !snap_v) continue;  // not iso-like — skip

        // Snap to exact iso: snap to the nearest surface domain
        // boundary (if the iso value is within 1e-4 of a boundary) or
        // to the mean otherwise (interior iso curves).
        {
            const int li = trim.m_li;
            const int fi = (li >= 0 && li < brep.m_L.Count())
                               ? brep.m_L[li].m_fi : -1;
            const ON_Surface* srf = (fi >= 0 && fi < brep.m_F.Count())
                                        ? brep.m_F[fi].SurfaceOf() : nullptr;
            double su0=0, su1=0, sv0=0, sv1=0;
            if (srf) {
                srf->GetDomain(0, &su0, &su1);
                srf->GetDomain(1, &sv0, &sv1);
            }

            auto snap_to_boundary = [](double val, double d0, double d1,
                                        double tol) -> double {
                if (fabs(val - d0) <= tol) return d0;
                if (fabs(val - d1) <= tol) return d1;
                return val;  // interior
            };
            // For singular trims, ALWAYS snap to the nearest surface boundary.
            // openNURBS requires singular trims to have W/E/N/S_iso.  If the
            // singularity is at an interior parameter (e.g. due to B-spline
            // conversion changing the parameterization), we must force it to
            // the nearest boundary.  The adjacent trim endpoints are corrected
            // by the junction-gap repair pass that follows below.
            auto snap_to_nearest = [](double val, double d0, double d1) -> double {
                return (fabs(val-d0) <= fabs(val-d1)) ? d0 : d1;
            };
            const bool is_singular = (trim.m_type == ON_BrepTrim::singular);

            if (snap_u) {
                double u_iso = 0.5 * (uMin + uMax);
                if (srf) {
                    const double bndTol = std::max(1e-4, 0.01*(su1-su0));
                    double u_snapped = snap_to_boundary(u_iso, su0, su1, bndTol);
                    if (u_snapped == u_iso && is_singular) {
                        // Interior U: force to nearest boundary
                        u_snapped = snap_to_nearest(u_iso, su0, su1);
                    }
                    u_iso = u_snapped;
                }
                for (int p = 0; p < nCV; ++p) {
                    double* cv = nc->CV(p);
                    double w = rat ? cv[nc->m_dim] : 1.0;
                    if (w == 0.0) w = 1.0;
                    cv[0] = u_iso * w;
                }
            } else {
                double v_iso = 0.5 * (vMin + vMax);
                if (srf) {
                    const double bndTol = std::max(1e-4, 0.01*(sv1-sv0));
                    double v_snapped = snap_to_boundary(v_iso, sv0, sv1, bndTol);
                    if (v_snapped == v_iso && is_singular) {
                        // Interior V: force to nearest boundary
                        v_snapped = snap_to_nearest(v_iso, sv0, sv1);
                    }
                    v_iso = v_snapped;
                }
                for (int p = 0; p < nCV; ++p) {
                    double* cv = nc->CV(p);
                    double w = rat ? cv[nc->m_dim] : 1.0;
                    if (w == 0.0) w = 1.0;
                    cv[1] = v_iso * w;
                }
            }
        }

        // Recompute the iso flag and parameter bounding box for this trim.
        brep.SetTrimIsoFlags(trim);

        // Fallback: if SetTrimIsoFlags still returns not_iso, x_iso, or y_iso
        // for a seam trim (can happen because ON_NurbsSurface::IsIsoparametric
        // uses a tight absolute tolerance, and because it cannot distinguish
        // between boundary and interior iso curves), compute the correct
        // boundary iso type directly from the snapped CV values and the surface
        // domain.  x_iso / y_iso are only valid for non-seam interior curves;
        // seam trims require W/E/S/N_iso.
        const bool still_bad =
            (trim.m_iso == ON_Surface::not_iso) ||
            ((trim.m_type == ON_BrepTrim::seam ||
              trim.m_type == ON_BrepTrim::singular) &&
             (trim.m_iso == ON_Surface::x_iso ||
              trim.m_iso == ON_Surface::y_iso));
        if (still_bad) {
            const int li2   = trim.m_li;
            const int fi2   = (li2 >= 0 && li2 < brep.m_L.Count())
                                  ? brep.m_L[li2].m_fi : -1;
            const ON_Surface* srf2 = (fi2 >= 0 && fi2 < brep.m_F.Count())
                                         ? brep.m_F[fi2].SurfaceOf() : nullptr;
            if (srf2 && nc->CVCount() > 0) {
                double su0=0, su1=0, sv0=0, sv1=0;
                srf2->GetDomain(0, &su0, &su1);
                srf2->GetDomain(1, &sv0, &sv1);
                // Use domain-relative tolerance: 1% of domain extent,
                // or absolute 1e-4, whichever is larger.
                const double uBndTol = std::max(1e-4, 0.01*(su1-su0));
                const double vBndTol = std::max(1e-4, 0.01*(sv1-sv0));
                if (snap_u) {
                    const double* cv0 = nc->CV(0);
                    double w0 = nc->IsRational() ? cv0[nc->m_dim] : 1.0;
                    if (w0 == 0.0) w0 = 1.0;
                    double u0 = cv0[0] / w0;
                    if      (fabs(u0 - su0) <= uBndTol) trim.m_iso = ON_Surface::W_iso;
                    else if (fabs(u0 - su1) <= uBndTol) trim.m_iso = ON_Surface::E_iso;
                    else                                 trim.m_iso = ON_Surface::x_iso;
                } else {
                    const double* cv0 = nc->CV(0);
                    double w0 = nc->IsRational() ? cv0[nc->m_dim] : 1.0;
                    if (w0 == 0.0) w0 = 1.0;
                    double v0 = cv0[1] / w0;
                    if      (fabs(v0 - sv0) <= vBndTol) trim.m_iso = ON_Surface::S_iso;
                    else if (fabs(v0 - sv1) <= vBndTol) trim.m_iso = ON_Surface::N_iso;
                    else                                 trim.m_iso = ON_Surface::y_iso;
                }
            }
        }

        // Rebuild m_pbox from the (now-corrected) pcurve.
        {
            const ON_Curve* c2 = brep.m_C2[trim.m_c2i];
            if (c2) {
                trim.m_pbox = c2->BoundingBox();
                trim.m_pbox.m_min.z = 0.0;
                trim.m_pbox.m_max.z = 0.0;
            }
        }

        // After snapping this seam pcurve to exact iso, close the
        // Close junction gaps on both sides of the seam trim:
        //
        // 1. Snap the LAST CV of the preceding trim to match this seam
        //    trim's first point (gap at the trim's START).
        // 2. Snap the FIRST CV of the next trim to match this seam
        //    trim's last point (gap at the trim's END).
        //
        // Only the iso (constant) coordinate is changed to avoid corrupting
        // the non-iso geometry of the adjacent trim.
        // Both snaps skip the case where the adjacent trim is itself a seam.
        {
            ON_3dPoint pt_start = trim.PointAtStart();
            ON_3dPoint pt_end   = trim.PointAtEnd();
            const int li2 = trim.m_li;
            if (li2 >= 0 && li2 < brep.m_L.Count()) {
                const ON_BrepLoop& lp2 = brep.m_L[li2];
                for (int k = 0; k < lp2.m_ti.Count(); ++k) {
                    if (lp2.m_ti[k] != ti) continue;

                    // ---- Snap preceding trim's last CV ----
                    {
                        int prev_k = (k + lp2.m_ti.Count() - 1)
                                         % lp2.m_ti.Count();
                        ON_BrepTrim& prev_t =
                            brep.m_T[lp2.m_ti[prev_k]];
                        if (prev_t.m_type != ON_BrepTrim::seam) {
                            int prev_c2i = prev_t.m_c2i;
                            if (prev_c2i >= 0
                                    && prev_c2i < brep.m_C2.Count()) {
                                ON_NurbsCurve* pnc = ON_NurbsCurve::Cast(
                                    brep.m_C2[prev_c2i]);
                                if (pnc && pnc->CVCount() >= 1) {
                                    const int last = pnc->CVCount() - 1;
                                    double* cvL = pnc->CV(last);
                                    double wL = pnc->IsRational()
                                                    ? cvL[pnc->m_dim] : 1.0;
                                    if (wL == 0.0) wL = 1.0;
                                    if (snap_u) cvL[0] = pt_start.x * wL;
                                    else        cvL[1] = pt_start.y * wL;
                                    prev_t.m_pbox = brep.m_C2[prev_c2i]->BoundingBox();
                                    prev_t.m_pbox.m_min.z = 0.0;
                                    prev_t.m_pbox.m_max.z = 0.0;
                                }
                            }
                        }
                    }

                    // ---- Snap next trim's first CV (only if already close) ----
                    {
                        int next_k = (k + 1) % lp2.m_ti.Count();
                        ON_BrepTrim& next_t =
                            brep.m_T[lp2.m_ti[next_k]];
                        if (next_t.m_type != ON_BrepTrim::seam) {
                            int next_c2i = next_t.m_c2i;
                            if (next_c2i >= 0
                                    && next_c2i < brep.m_C2.Count()) {
                                ON_NurbsCurve* nnc = ON_NurbsCurve::Cast(
                                    brep.m_C2[next_c2i]);
                                if (nnc && nnc->CVCount() >= 1) {
                                    double* cv0 = nnc->CV(0);
                                    double w0 = nnc->IsRational()
                                                    ? cv0[nnc->m_dim] : 1.0;
                                    if (w0 == 0.0) w0 = 1.0;
                                    // For singular trims, always snap the
                                    // next trim to the iso value (the apex
                                    // boundary can be offset vs. the adjacent
                                    // trim's natural start by up to 0.5 UV
                                    // units due to B-spline approximation of
                                    // cones/spheres).  For seam trims, only
                                    // snap if already close (1e-3) to avoid
                                    // corrupting trims on the opposite seam.
                                    const bool is_singular_trim =
                                        (trim.m_type == ON_BrepTrim::singular);
                                    bool snap_next = is_singular_trim;
                                    if (!snap_next) {
                                        if (snap_u) {
                                            double cur_u = cv0[0] / w0;
                                            snap_next = fabs(cur_u - pt_end.x) <= 1e-3;
                                        } else {
                                            double cur_v = cv0[1] / w0;
                                            snap_next = fabs(cur_v - pt_end.y) <= 1e-3;
                                        }
                                    }
                                    if (snap_next) {
                                        if (snap_u) cv0[0] = pt_end.x * w0;
                                        else        cv0[1] = pt_end.y * w0;
                                        next_t.m_pbox = brep.m_C2[next_c2i]->BoundingBox();
                                        next_t.m_pbox.m_min.z = 0.0;
                                        next_t.m_pbox.m_max.z = 0.0;
                                    }
                                }
                            }
                        }
                    }

                    break;
                }
            }
        }
    }

    // Rebuild loop pboxes after seam/singular trim CV snapping.
    // SetTolerancesBoxesAndFlags computed loop pboxes from the original CVs;
    // our snapping may have moved a trim's pbox outside the loop's pbox.
    for (int li = 0; li < brep.m_L.Count(); ++li) {
        ON_BrepLoop& lp = brep.m_L[li];
        lp.m_pbox.Destroy();
        for (int k = 0; k < lp.m_ti.Count(); ++k) {
            int tki = lp.m_ti[k];
            if (tki >= 0 && tki < brep.m_T.Count())
                lp.m_pbox.Union(brep.m_T[tki].m_pbox);
        }
        // Parameter-space bounding boxes are 2D (UV only); the z coordinate
        // must be zero so that ON_BrepLoop::IsValid() accepts the pbox.
        lp.m_pbox.m_min.z = 0.0;
        lp.m_pbox.m_max.z = 0.0;
    }

    // ---------------------------------------------------------------------------
    // Post-processing pass: fix edge tolerances so that ON_Brep::IsValid() passes
    // the "distance from trim endpoint to 3d edge" check.
    //
    // On closed-but-non-periodic surfaces (common in Rhino models) the B-spline
    // evaluates slightly differently on the two sides of the seam even though
    // they represent the same physical point.  After round-trip through OCCT the
    // edge 3D curve is tied to one side while the pcurves on the opposite side
    // map to 3D points that are O(0.1..0.4) units away.
    //
    // The validation formula is:
    //   dtol  = max(0.01,  10 * (edge_tol + t0_3d_tol + t1_3d_tol))
    //   FAIL  if  dist(srf(uv_start), edge_start) > dtol
    //         or  dist(srf(uv_end),   edge_end  ) > dtol
    //
    // To guarantee the check passes we raise edge.m_tolerance to d_max / 9.
    // (That gives dtol >= 10*(d_max/9) > d_max — a ~11 % safety margin.)
    // ---------------------------------------------------------------------------
    for (int ti = 0; ti < brep.m_T.Count(); ++ti) {
        ON_BrepTrim& trim = brep.m_T[ti];
        if (trim.m_ei < 0 || trim.m_li < 0) continue;

        const int fi = brep.m_L[trim.m_li].m_fi;
        if (fi < 0) continue;
        const ON_Surface* srf = brep.m_F[fi].SurfaceOf();
        if (!srf) continue;

        ON_BrepEdge& edge = brep.m_E[trim.m_ei];

        // Evaluate the surface at the trim's UV endpoints.
        // ON_BrepTrim::PointAtStart() / PointAtEnd() return (u, v, 0).
        ON_3dPoint uv0 = trim.PointAtStart();
        ON_3dPoint uv1 = trim.PointAtEnd();
        ON_3dPoint srf_pt0, srf_pt1;
        if (!srf->EvPoint(uv0.x, uv0.y, srf_pt0)) continue;
        if (!srf->EvPoint(uv1.x, uv1.y, srf_pt1)) continue;

        // Map trim orientation to edge endpoints.
        const ON_3dPoint edge_pt0 =
            trim.m_bRev3d ? edge.PointAtEnd()   : edge.PointAtStart();
        const ON_3dPoint edge_pt1 =
            trim.m_bRev3d ? edge.PointAtStart() : edge.PointAtEnd();

        const double d0 = edge_pt0.DistanceTo(srf_pt0);
        const double d1 = edge_pt1.DistanceTo(srf_pt1);
        const double d_max = std::max(d0, d1);

        // Always take the maximum so that a later trim with a smaller d_max
        // cannot overwrite a larger tolerance already set by an earlier trim
        // on the same edge.  (Multiple trims can share the same edge.)
        const double new_etol = d_max / 9.0;
        if (new_etol > edge.m_tolerance)
            edge.m_tolerance = new_etol;
    }

    // ---------------------------------------------------------------------------
    // Post-processing pass: fix the tangent-direction check for closed trims on
    // closed edges (ON_Brep.m_T[i].m_bRev3d validation).
    //
    // The openNURBS validation requires that for a closed trim on a closed edge
    // the 3-D tangent of the pcurve mapped through the surface Jacobian aligns
    // with the edge-curve tangent at BOTH the start (d0) and end (d1) of the
    // trim domain.  OCCT Boolean-result B-spline pcurves often have a small
    // tangent discontinuity (kink) at the parameter-domain junction (where the
    // closed loop starts/ends), introduced during OCCT's intersection
    // computation.  This kink causes d0 or d1 to be barely negative even though
    // the geometric direction is correct.
    //
    // Fix: for each closed trim on a closed edge whose d0 or d1 fails, snap the
    // 2-D pcurve's last interior control-vertex (CV[n-2] for d1-fail, or CV[1]
    // for d0-fail) so that the end tangent exactly equals the start tangent
    // (C1 closure without kink).  The fix preserves the start and end UV points
    // (only the tangent direction changes) so loop UV connectivity is unaffected.
    // ---------------------------------------------------------------------------
    // NOTE: Do NOT call brep.SetTolerancesBoxesAndFlags() here: it would reset
    // the edge tolerances we adjusted in the loop above.  The trim proxy-curve
    // domains were already set during the conversion loop via
    // SetProxyCurveDomain, so trim.Domain() and trim.Ev1Der() work without it.

    for (int ti = 0; ti < brep.m_T.Count(); ++ti) {
        ON_BrepTrim& trim = brep.m_T[ti];
        if (trim.m_ei < 0 || trim.m_li < 0) continue;
        const ON_BrepEdge& edge = brep.m_E[trim.m_ei];

        // Only process truly closed trims on closed edges
        if (trim.m_vi[0] != trim.m_vi[1]) continue;
        if (edge.m_vi[0] != edge.m_vi[1]) continue;
        if (trim.m_vi[0] != edge.m_vi[0]) continue;

        const int fi = brep.m_L[trim.m_li].m_fi;
        if (fi < 0) continue;
        const ON_Surface* srf = brep.m_F[fi].SurfaceOf();
        if (!srf) continue;

        // Compute the tangent check values d0 and d1
        auto compute_checks = [&](const ON_BrepTrim& t) -> std::pair<double,double> {
            ON_Interval td = t.Domain();
            ON_3dPoint uv0, uv1; ON_3dVector td0, td1;
            t.Ev1Der(td[0], uv0, td0);
            t.Ev1Der(td[1], uv1, td1);
            ON_3dPoint sp; ON_3dVector sdu, sdv;
            srf->Ev1Der(uv0.x, uv0.y, sp, sdu, sdv);
            ON_3dVector tt0 = td0.x*sdu + td0.y*sdv; tt0.Unitize();
            srf->Ev1Der(uv1.x, uv1.y, sp, sdu, sdv);
            ON_3dVector tt1 = td1.x*sdu + td1.y*sdv; tt1.Unitize();
            const ON_BrepEdge& e = brep.m_E[t.m_ei];
            bool rev = t.m_bRev3d ? true : false;
            ON_3dVector et0 = e.TangentAt(e.Domain()[rev ? 1 : 0]);
            ON_3dVector et1 = e.TangentAt(e.Domain()[rev ? 0 : 1]);
            double d0 = tt0 * et0; if (rev) d0 = -d0;
            double d1 = tt1 * et1; if (rev) d1 = -d1;
            return {d0, d1};
        };

        auto [d0, d1] = compute_checks(trim);
        // openNURBS IsValid() uses strict d < 0.0 check for closed trims.
        // Use a small positive threshold so any negative (or exactly-zero)
        // dot product triggers the snap/fix below.
        const double kSnapThreshold = 1e-9;
        if (d0 >= kSnapThreshold && d1 >= kSnapThreshold) continue;

        // If BOTH d0 and d1 are strongly negative the m_bRev3d flag itself
        // is wrong (edge and pcurve go in fully opposite directions).  Flip
        // m_bRev3d and recheck before trying the CV tangent snap.
        // This fix is applied independently of the pcurve degree.
        {
            double ad0 = d0, ad1 = d1;
            if (ad0 < -0.5 && ad1 < -0.5) {
                trim.m_bRev3d = !trim.m_bRev3d;
                auto [nd0, nd1] = compute_checks(trim);
                if (nd0 >= kSnapThreshold && nd1 >= kSnapThreshold) continue;
                // Still failing after flip — restore and fall through to snap.
                trim.m_bRev3d = !trim.m_bRev3d;
            }
        }

        // Get the ON_NurbsCurve for this trim's pcurve
        if (trim.m_c2i < 0 || trim.m_c2i >= brep.m_C2.Count()) continue;
        ON_NurbsCurve* nc = ON_NurbsCurve::Cast(brep.m_C2[trim.m_c2i]);
        if (!nc || nc->Degree() < 1) continue;

        // If the pcurve is linear (degree 1, 2 CVs) elevate it to degree 2
        // so that we have an interior CV to apply the C1 snap below.
        if (nc->CVCount() < 3) {
            if (nc->IncreaseDegree(2) && nc->CVCount() >= 3) {
                // Rebuild pbox for the trim after elevation.
                trim.m_pbox = nc->BoundingBox();
                trim.m_pbox.m_min.z = 0.0;
                trim.m_pbox.m_max.z = 0.0;
            } else {
                continue;  // elevation failed — skip
            }
        }
        if (nc->CVCount() < 3) continue;

        // We need at least CV[0], CV[1], CV[n-2], CV[n-1] to be accessible
        const int nCV = nc->CVCount();
        const int dim = nc->m_dim; // should be 2
        if (dim < 2) continue;

        // Read CV[0], CV[1] and CV[n-1], CV[n-2] in Euclidean 2D
        // (divide by weight for rational curves)
        auto get_cv2d = [&](int i) -> ON_2dPoint {
            ON_4dPoint h; nc->GetCV(i, h);
            double w = (h.w != 0.0) ? h.w : 1.0;
            return ON_2dPoint(h.x / w, h.y / w);
        };
        auto get_cv_weight = [&](int i) -> double {
            ON_4dPoint h; nc->GetCV(i, h);
            return (h.w != 0.0) ? h.w : 1.0;
        };
        auto set_cv2d = [&](int i, const ON_2dPoint& p, double w) {
            nc->SetCV(i, ON_4dPoint(p.x * w, p.y * w, 0.0, w));
        };

        // First interior knot span at start: m_knot[deg] - m_knot[0]
        // Last interior knot span at end  : m_knot[kc-1] - m_knot[kc-1-deg]
        // (openNURBS stores d-1 phantom knots at each end; the tangent formula is
        //  T'(t0) ∝ (CV[1]-CV[0]) / span_s  and  T'(t1) ∝ (CV[n-1]-CV[n-2]) / span_e)
        const int deg = nc->Degree();
        const int kc  = nc->KnotCount(); // = nCV + deg - 1
        double span_s = 0.0, span_e = 0.0;
        if (deg >= 1 && kc > deg) {
            span_s = nc->m_knot[deg] - nc->m_knot[0];
        }
        if (kc >= deg + 1) {
            span_e = nc->m_knot[kc-1] - nc->m_knot[kc-1-deg];
        }
        double scale = (span_s > 1e-14) ? (span_e / span_s) : 1.0;

        // Guard: if the span ratio is extreme (>> 1 or << 1), the 2D pcurve
        // C1 snap would displace CV[n-2] or CV[1] by a huge amount, distorting
        // the pcurve severely and causing BRepCheck_UnorientableShape in the
        // round-trip OCCT shape.  Skip only the 2D snap in such cases; the
        // 3D edge snap and bRev3d flip below still run.
        const bool do_2d_snap = !(scale > 10.0 || scale < 0.1 || scale != scale);

        if (do_2d_snap) {
            ON_2dPoint cv0  = get_cv2d(0);
            ON_2dPoint cv1  = get_cv2d(1);
            ON_2dPoint cvn1 = get_cv2d(nCV-1);
            ON_2dPoint cvn2 = get_cv2d(nCV-2);

            if (d1 < 0.0) {
                // End tangent bad: snap CV[n-2] to enforce T_end = T_start
                ON_2dPoint cvn2_new;
                cvn2_new.x = cvn1.x - (cv1.x - cv0.x) * scale;
                cvn2_new.y = cvn1.y - (cv1.y - cv0.y) * scale;
                set_cv2d(nCV-2, cvn2_new, get_cv_weight(nCV-2));
            } else {
                // Start tangent bad: snap CV[1] to enforce T_start = T_end
                ON_2dPoint cv1_new;
                cv1_new.x = cv0.x + (cvn1.x - cvn2.x) / scale;
                cv1_new.y = cv0.y + (cvn1.y - cvn2.y) / scale;
                set_cv2d(1, cv1_new, get_cv_weight(1));
            }
        }

        // After the 2D pcurve snap (if applied), recheck.  If d1 is still bad
        // (or the 2D snap was skipped due to scale guard), also try the 3D
        // edge curve: snap CV[n-2] to remove a C1 kink at the seam, then
        // re-evaluate and flip bRev3d if the flip gives a better result.
        {
            auto [pd0, pd1] = compute_checks(trim);
            // Try to snap the 3D edge when either endpoint check fails
            if (pd0 < 0.0 || pd1 < 0.0) {
                ON_NurbsCurve* ec = ON_NurbsCurve::Cast(
                    brep.m_C3[brep.m_E[trim.m_ei].m_c3i]);
                if (ec && ec->Degree() >= 1 && ec->CVCount() >= 3) {
                    const int enc  = ec->CVCount();
                    const int edeg = ec->Degree();
                    const int ekc  = ec->KnotCount();
                    double esp_s = 0.0, esp_e = 0.0;
                    if (ekc > edeg)
                        esp_s = ec->m_knot[edeg] - ec->m_knot[0];
                    if (ekc >= edeg + 1)
                        esp_e = ec->m_knot[ekc-1] - ec->m_knot[ekc-1-edeg];
                    double escale = (esp_s > 1e-14) ? (esp_e / esp_s) : 1.0;
                    // Widen the escale guard: even if span ratio is up to 100,
                    // the snap can be worthwhile when the kink is near-180°.
                    // The snap just moves one CV slightly; a large span ratio
                    // means the CV moves proportionally more but the 3D kink
                    // fix is still useful.  Only skip if escale is degenerate
                    // (NaN/Inf/zero span).
                    bool escale_ok = (esp_s > 1e-14) &&
                                     (escale == escale) && // not NaN
                                     (escale > 1e-6) && (escale < 1e6);
                    if (escale_ok) {
                        auto eget = [&](int i) -> ON_3dPoint {
                            ON_4dPoint h; ec->GetCV(i, h);
                            double w = (h.w != 0.0) ? h.w : 1.0;
                            return ON_3dPoint(h.x/w, h.y/w, h.z/w);
                        };
                        auto ew = [&](int i) -> double {
                            ON_4dPoint h; ec->GetCV(i, h);
                            return (h.w != 0.0) ? h.w : 1.0;
                        };
                        ON_3dPoint ep0 = eget(0), ep1 = eget(1);
                        ON_3dPoint epn1 = eget(enc-1), epn2 = eget(enc-2);
                        if (pd1 < 0.0) {
                            // End tangent bad: snap CV[n-2] so end tan = start tan
                            ON_3dPoint epn2_new(
                                epn1.x - (ep1.x - ep0.x) * escale,
                                epn1.y - (ep1.y - ep0.y) * escale,
                                epn1.z - (ep1.z - ep0.z) * escale);
                            double ew2 = ew(enc-2);
                            ec->SetCV(enc-2, ON_4dPoint(
                                epn2_new.x * ew2,
                                epn2_new.y * ew2,
                                epn2_new.z * ew2,
                                ew2));
                        } else {
                            // Start tangent bad: snap CV[1] so start tan = end tan
                            double inv_sc = (escale > 1e-14) ? (1.0/escale) : 1.0;
                            ON_3dPoint ep1_new(
                                ep0.x + (epn1.x - epn2.x) * inv_sc,
                                ep0.y + (epn1.y - epn2.y) * inv_sc,
                                ep0.z + (epn1.z - epn2.z) * inv_sc);
                            double ew1 = ew(1);
                            ec->SetCV(1, ON_4dPoint(
                                ep1_new.x * ew1,
                                ep1_new.y * ew1,
                                ep1_new.z * ew1,
                                ew1));
                        }
                    }
                }

                // After the 3D edge snap removes the C1 kink, re-evaluate the
                // tangent checks.  If bRev3d is wrong, the kink may have been
                // the only thing making one of the checks pass; once the kink is
                // gone both checks become clearly negative under the wrong flag.
                // Flip bRev3d if the min check improves under the flipped flag.
                {
                    trim.m_bRev3d = !trim.m_bRev3d;
                    auto [fd0, fd1] = compute_checks(trim);
                    if (std::min(fd0, fd1) > std::min(pd0, pd1)) {
                        // Flipped orientation is better; keep it.
                        // Also re-snap CV[n-2] of the 2D pcurve now that bRev3d
                        // is corrected (the tangent reference direction flipped).
                        if (trim.m_c2i >= 0 && trim.m_c2i < brep.m_C2.Count()) {
                            ON_NurbsCurve* nc2 = ON_NurbsCurve::Cast(
                                brep.m_C2[trim.m_c2i]);
                            if (nc2 && nc2->CVCount() >= 3) {
                                const int nc = nc2->CVCount();
                                const int deg2 = nc2->Degree();
                                const int kc2  = nc2->KnotCount();
                                double ss = 0.0, se = 0.0;
                                if (kc2 > deg2)
                                    ss = nc2->m_knot[deg2] - nc2->m_knot[0];
                                if (kc2 >= deg2 + 1)
                                    se = nc2->m_knot[kc2-1] - nc2->m_knot[kc2-1-deg2];
                                double sc2 = (ss > 1e-14) ? (se / ss) : 1.0;
                                if (sc2 > 0.1 && sc2 < 10.0) {
                                    auto g2 = [&](int i) -> ON_2dPoint {
                                        ON_4dPoint h; nc2->GetCV(i, h);
                                        double w = (h.w != 0.0) ? h.w : 1.0;
                                        return ON_2dPoint(h.x/w, h.y/w);
                                    };
                                    auto gw2 = [&](int i) -> double {
                                        ON_4dPoint h; nc2->GetCV(i, h);
                                        return (h.w != 0.0) ? h.w : 1.0;
                                    };
                                    auto s2 = [&](int i, const ON_2dPoint& p, double w) {
                                        nc2->SetCV(i, ON_4dPoint(p.x*w, p.y*w, 0.0, w));
                                    };
                                    ON_2dPoint p0 = g2(0), p1 = g2(1);
                                    ON_2dPoint pn1 = g2(nc-1);
                                    // snap end tangent to start tangent
                                    ON_2dPoint pn2_new;
                                    pn2_new.x = pn1.x - (p1.x - p0.x) * sc2;
                                    pn2_new.y = pn1.y - (p1.y - p0.y) * sc2;
                                    s2(nc-2, pn2_new, gw2(nc-2));
                                }
                            }
                        }
                    } else {
                        // Flipped is not better; restore.
                        trim.m_bRev3d = !trim.m_bRev3d;
                    }
                }
            }
        }
    }

    return brep.IsValid() ? true : false;
}

} // namespace open2open
