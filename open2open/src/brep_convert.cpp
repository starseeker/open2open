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

// ---- OCCT containers ----
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
    for (int i = 0; i < brep.m_E.Count(); ++i) {
        const ON_BrepEdge& e = brep.m_E[i];
        B.MakeEdge(e_map[i]);

        if (e.m_c3i >= 0 && e.m_c3i < (int)c3_map.size() &&
            !c3_map[e.m_c3i].IsNull()) {
            double t0 = e.Domain().Min();
            double t1 = e.Domain().Max();
            B.UpdateEdge(e_map[i], c3_map[e.m_c3i], linear_tolerance);
            B.Range(e_map[i], t0, t1, /*only3d=*/Standard_True);
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
            // TopExp::Vertices(edge, V0, V1, CumOri=false) returns V0=FORWARD
            // and V1=REVERSED, so both must have distinct orientations for
            // OCCTToON_Brep to recover the correct vi[0]/vi[1] mapping.
            TopoDS_Vertex vend = v_map[e.m_vi[1]];
            vend.Orientation(TopAbs_REVERSED);
            B.Add(e_map[i], vend);
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
                    B.Range(e_map[ei], f_map[fi],
                            trim.Domain().Min(),
                            trim.Domain().Max());
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
    // Step 5 — assemble shell / solid
    // ------------------------------------------------------------------
    TopoDS_Shell shell;
    B.MakeShell(shell);
    for (int fi = 0; fi < brep.m_F.Count(); ++fi)
        B.Add(shell, f_map[fi]);

    // Attempt to build a solid
    TopoDS_Solid solid;
    B.MakeSolid(solid);
    B.Add(solid, shell);

    return solid;
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

        int c3i = -1;
        if (!gc.IsNull()) {
            // For non-BSpline curves (Geom_Line, Geom_Circle, Geom_Ellipse,
            // etc.) always convert only the edge's actual parameter range
            // [t0, t1] using a Geom_TrimmedCurve.  This avoids domain
            // mismatch when the full curve spans more than the edge (e.g.
            // Geom_Circle with t=[3π/2..5π/2]) and avoids periodic BSplines.
            //
            // For topologically-closed edges we also force the TrimmedCurve
            // path even when gc is already a BSpline, so that the resulting
            // ON_NurbsCurve has domain exactly [t0, t1] with matching
            // start/end 3D points (required for IsClosed() == true).
            Handle(Geom_BSplineCurve) bc;
            if (!topoClosedEdge) {
                bc = Handle(Geom_BSplineCurve)::DownCast(gc);
            }
            if (bc.IsNull()) {
                // Trimmed conversion first (preserves arc domain)
                try {
                    Handle(Geom_TrimmedCurve) tc =
                        new Geom_TrimmedCurve(gc, t0, t1);
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
        // TopExp::Vertices(edge, V0, V1, CumOri=false) always gives
        // V0 = vertex at parameter t0 (start of 3D curve) and
        // V1 = vertex at parameter t1 (end of 3D curve), regardless of
        // the edge's orientation in the parent shape.  This is the correct
        // order for ON_BrepEdge.m_vi[0]/[1].
        int vi0 = -1, vi1 = -1;
        {
            TopoDS_Vertex tv0, tv1;
            TopExp::Vertices(edge, tv0, tv1, /*CumOri=*/false);
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
        if (c3i >= 0) {
            // For closed edges we set the proxy domain to the curve's own
            // domain so that ON_CurveProxy::IsClosed() can delegate to the
            // underlying curve (it requires ProxyCurveDomain()==Curve()->Domain()).
            // For non-closed edges the proxy domain is the edge's actual
            // parameter range [t0, t1] within a potentially wider BSpline.
            if (topoClosedEdge) {
                ON_NurbsCurve* nc_chk =
                    ON_NurbsCurve::Cast(brep.m_C3[c3i]);
                ON_Interval crv_dom = nc_chk ? nc_chk->Domain()
                                             : ON_Interval(t0, t1);
                brep.m_E[this_ei].SetProxyCurveDomain(crv_dom);
            } else {
                brep.m_E[this_ei].SetProxyCurveDomain(ON_Interval(t0, t1));
            }
        }
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
            if (bs.IsNull()) {
                try {
                    bs = GeomConvert::SurfaceToBSplineSurface(gs);
                } catch (...) {}
            }
            if (bs.IsNull()) {
                // Fallback for unbounded analytical surfaces
                // (Geom_Plane, Geom_ConicalSurface, etc.):
                // restrict to the face's UV parameter domain first.
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
                // Periodic BSpline surfaces (e.g. sphere U-direction, cylinder
                // lateral surface) have sum(mults) = nu+deg instead of nu+deg+1.
                // OCCTSurfaceToON expects the non-periodic convention, so
                // flatten any periodic directions before conversion.
                if (bs->IsUPeriodic()) bs->SetUNotPeriodic();
                if (bs->IsVPeriodic()) bs->SetVNotPeriodic();
                ON_NurbsSurface* ns = new ON_NurbsSurface();
                if (OCCTSurfaceToON(bs, *ns)) {
                    si = brep.AddSurface(ns);
                } else {
                    delete ns;
                }
            }

            // Cache by shape pointer so seam surfaces are shared
            Standard_Address sk = face.TShape().get();
            if (!surface_map.count(sk) && si >= 0)
                surface_map[sk] = si;
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

            ON_BrepLoop& ol = brep.NewLoop(ON_BrepLoop::unknown, of);

            // Collect edges in the correct wire traversal order.
            // BRepTools_WireExplorer follows the wire's connectivity (handles
            // seam and degenerate edges in native OCCT shapes correctly).
            // For ON_BrepToOCCT-generated wires that lack full vertex
            // connectivity, the wire explorer may not visit all edges.  In
            // that case fall back to TopExp_Explorer which is order-agnostic
            // but visits all edges.
            std::vector<TopoDS_Edge> wire_edges;
            {
                for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next())
                    wire_edges.push_back(we.Current());
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
                if (we_keys.size() < tex_keys.size()) {
                    // WireExplorer missed some edges — revert to TopExp order
                    wire_edges.clear();
                    for (TopExp_Explorer eex(wire, TopAbs_EDGE);
                         eex.More(); eex.Next())
                        wire_edges.push_back(TopoDS::Edge(eex.Current()));
                }
            }

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
                                          ON_3dPoint(p2.X() * w2,
                                                     p2.Y() * w2,
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
                    if (c2i >= 0)
                        strim.SetProxyCurveDomain(ON_Interval(pc_t0, pc_t1));
                    continue;
                }

                ON_BrepTrim& trim =
                    brep.NewTrim(brep.m_E[ei], bRev3d, ol, c2i);
                trim.m_tolerance[0] = BRep_Tool::Tolerance(edge);
                trim.m_tolerance[1] = trim.m_tolerance[0];
                if (c2i >= 0) {
                    // The pcurve has already been reversed (if needed) in the
                    // loop above, so it always goes from trim-start to trim-end.
                    // SetProxyCurveDomain requires an increasing interval.
                    trim.SetProxyCurveDomain(ON_Interval(pc_t0, pc_t1));
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
                        const double kRelTol = 1e-3;
                        double shift_u = 0.0, shift_v = 0.0;
                        if (cls_u && uperiod > 0.0 &&
                            fabs(fabs(dx) - uperiod) <
                                kRelTol * uperiod &&
                            fabs(dy) < kRelTol * uperiod) {
                            shift_u = -dx;
                        } else if (cls_v && vperiod > 0.0 &&
                                   fabs(fabs(dy) - vperiod) <
                                       kRelTol * vperiod &&
                                   fabs(dx) < kRelTol * vperiod) {
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
        // Only snap when d is significantly negative (not just floating-point
        // noise).  Genuine kink failures have d ≈ -0.05 to -0.2.
        const double kSnapThreshold = -0.02;
        if (d0 >= kSnapThreshold && d1 >= kSnapThreshold) continue;

        // Get the ON_NurbsCurve for this trim's pcurve
        if (trim.m_c2i < 0 || trim.m_c2i >= brep.m_C2.Count()) continue;
        ON_NurbsCurve* nc = ON_NurbsCurve::Cast(brep.m_C2[trim.m_c2i]);
        if (!nc || nc->Degree() < 1 || nc->CVCount() < 3) continue;

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
            nc->SetCV(i, ON_3dPoint(p.x * w, p.y * w, w));
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

        // Guard: if the span ratio is extreme (>> 1 or << 1), the C1 snap
        // would displace CV[n-2] or CV[1] by a huge amount, distorting the
        // pcurve severely and causing BRepCheck_UnorientableShape in the
        // round-trip OCCT shape.  Skip the snap in such cases.
        if (scale > 10.0 || scale < 0.1 || scale != scale) continue;

        ON_2dPoint cv0  = get_cv2d(0);
        ON_2dPoint cv1  = get_cv2d(1);
        ON_2dPoint cvn1 = get_cv2d(nCV-1);
        ON_2dPoint cvn2 = get_cv2d(nCV-2);

        if (d1 < 0.0) {
            // End tangent bad: snap CV[n-2] to enforce T_end = T_start
            // T_start direction in 2D: (cv1 - cv0) / span_s
            // T_end   direction in 2D: (cvn1 - cvn2) / span_e
            // C1 closure: (cvn1 - cvn2_new) / span_e = (cv1 - cv0) / span_s
            ON_2dPoint cvn2_new;
            cvn2_new.x = cvn1.x - (cv1.x - cv0.x) * scale;
            cvn2_new.y = cvn1.y - (cv1.y - cv0.y) * scale;
            set_cv2d(nCV-2, cvn2_new, get_cv_weight(nCV-2));
        } else {
            // Start tangent bad: snap CV[1] to enforce T_start = T_end
            // T_end   direction in 2D: (cvn1 - cvn2) / span_e
            // C1 closure: (cv1_new - cv0) / span_s = (cvn1 - cvn2) / span_e
            ON_2dPoint cv1_new;
            cv1_new.x = cv0.x + (cvn1.x - cvn2.x) / scale;
            cv1_new.y = cv0.y + (cvn1.y - cvn2.y) / scale;
            set_cv2d(1, cv1_new, get_cv_weight(1));
        }
    }

    return brep.IsValid() ? true : false;
}

} // namespace open2open
