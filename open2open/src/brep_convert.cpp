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

#include <vector>
#include <map>
#include <set>

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
        if (e.m_vi[0] >= 0 && e.m_vi[0] < (int)v_map.size())
            B.Add(e_map[i], v_map[e.m_vi[0]]);
        if (e.m_vi[1] >= 0 && e.m_vi[1] < (int)v_map.size() &&
            e.m_vi[1] != e.m_vi[0])
            B.Add(e_map[i], v_map[e.m_vi[1]]);
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
                B.UpdateEdge(e_map[ei],
                             c2_map[sp.c2i_fwd],
                             c2_map[sp.c2i_rev],
                             f_map[fi],
                             sp.tol);
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

    // Maps from OCCT shape (by pointer/hash) to openNURBS index
    std::map<Standard_Address, int> vertex_map;  // TShape* → vertex index
    std::map<Standard_Address, int> edge_map;    // TShape* → edge index
    std::map<Standard_Address, int> surface_map; // TShape* → surface index

    // ------------------------------------------------------------------
    // Pass 1 — collect and translate all vertices
    // ------------------------------------------------------------------
    for (TopExp_Explorer ex(shape, TopAbs_VERTEX); ex.More(); ex.Next()) {
        const TopoDS_Vertex& vtx =
            TopoDS::Vertex(ex.Current());
        Standard_Address key = vtx.TShape().get();
        if (vertex_map.count(key)) continue;

        gp_Pnt p = BRep_Tool::Pnt(vtx);
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
        Standard_Address key = edge.TShape().get();
        if (edge_map.count(key)) continue;

        // Degenerate edges (poles) must NOT become ON_BrepEdge objects —
        // they are represented as singular trims in openNURBS.  We store
        // a sentinel value (-2) so Pass 3 can detect them.
        if (BRep_Tool::Degenerated(edge)) {
            edge_map[key] = -2;
            continue;
        }

        double t0, t1;
        Handle(Geom_Curve) gc = BRep_Tool::Curve(edge, t0, t1);

        int c3i = -1;
        if (!gc.IsNull()) {
            // Convert to BSpline.  For analytical curves (Geom_Line,
            // Geom_Circle, Geom_Ellipse, …) that are not bounded,
            // wrap in a Geom_TrimmedCurve using the edge parameter range
            // before calling GeomConvert so that the result is finite.
            Handle(Geom_BSplineCurve) bc =
                Handle(Geom_BSplineCurve)::DownCast(gc);
            if (bc.IsNull()) {
                try {
                    bc = GeomConvert::CurveToBSplineCurve(gc);
                } catch (...) {}
            }
            if (bc.IsNull()) {
                // Fallback: use the edge parameter range to bound the curve
                try {
                    Handle(Geom_TrimmedCurve) tc =
                        new Geom_TrimmedCurve(gc, t0, t1);
                    bc = GeomConvert::CurveToBSplineCurve(tc);
                } catch (...) {}
            }
            if (!bc.IsNull()) {
                ON_NurbsCurve* nc = new ON_NurbsCurve();
                if (OCCTCurveToON(bc, *nc)) {
                    c3i = brep.AddEdgeCurve(nc);
                } else {
                    delete nc;
                }
            }
        }

        // Determine start/end vertex indices.
        // Force FORWARD orientation on the edge copy so that
        // TopExp_Explorer(edge, VERTEX) always gives vi[0] first and vi[1]
        // second, regardless of how the edge is oriented in the parent shape.
        // (For ON_BrepToOCCT-created edges both vertices are stored FORWARD;
        // for native OCCT edges the start vertex is FORWARD and the end is
        // REVERSED.  Both cases work with a FORWARD-forced copy.)
        int vi0 = -1, vi1 = -1;
        {
            TopoDS_Edge fwd_edge = edge;
            fwd_edge.Orientation(TopAbs_FORWARD);
            TopExp_Explorer vex(fwd_edge, TopAbs_VERTEX);
            if (vex.More()) {
                Standard_Address vk = vex.Current().TShape().get();
                auto it = vertex_map.find(vk);
                if (it != vertex_map.end()) vi0 = it->second;
                vex.Next();
            }
            if (vex.More()) {
                Standard_Address vk = vex.Current().TShape().get();
                auto it = vertex_map.find(vk);
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
            brep.m_E[this_ei].SetProxyCurveDomain(ON_Interval(t0, t1));
        }
        edge_map[key] = this_ei;
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
        for (TopExp_Explorer wex(face, TopAbs_WIRE); wex.More(); wex.Next()) {
            const TopoDS_Wire& wire = TopoDS::Wire(wex.Current());

            ON_BrepLoop& ol = brep.NewLoop(ON_BrepLoop::unknown, of);

            for (TopExp_Explorer eex(wire, TopAbs_EDGE); eex.More(); eex.Next()) {
                const TopoDS_Edge& edge = TopoDS::Edge(eex.Current());
                Standard_Address ek = edge.TShape().get();

                auto eit = edge_map.find(ek);
                if (eit == edge_map.end()) continue;
                int ei = eit->second;

                // Translate 2D pcurve for this edge on this face
                double pc_t0, pc_t1;
                Handle(Geom2d_Curve) gc2 =
                    BRep_Tool::CurveOnSurface(edge, face, pc_t0, pc_t1);

                int c2i = -1;
                if (!gc2.IsNull()) {
                    // Convert 2D pcurve to B-spline.  For analytical curves
                    // (Geom2d_Line, Geom2d_Circle, etc.) that are unbounded,
                    // wrap in a Geom2d_TrimmedCurve using the pcurve domain.
                    Handle(Geom2d_BSplineCurve) bc2 =
                        Handle(Geom2d_BSplineCurve)::DownCast(gc2);
                    if (bc2.IsNull()) {
                        try {
                            bc2 = Geom2dConvert::CurveToBSplineCurve(gc2);
                        } catch (...) {}
                    }
                    if (bc2.IsNull()) {
                        try {
                            Handle(Geom2d_TrimmedCurve) tc2 =
                                new Geom2d_TrimmedCurve(gc2, pc_t0, pc_t1);
                            bc2 = Geom2dConvert::CurveToBSplineCurve(tc2);
                        } catch (...) {}
                    }
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
                                // OCCT convention: pcurves are stored in the
                                // edge direction (edge.vi[0] → vi[1]).  For
                                // REVERSED (non-seam) edges the openNURBS trim
                                // direction is opposite, so the pcurve must be
                                // reversed to go from trim-start to trim-end.
                                // Seam edges are exempt: OCCT stores a
                                // separate pcurve for each traversal direction
                                // via the double-pcurve UpdateEdge overload,
                                // so the returned pcurve is already oriented
                                // in the trim direction.
                                bool need_rev =
                                    (edge.Orientation() == TopAbs_REVERSED) &&
                                    !BRep_Tool::IsClosed(edge, face);
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

            // ---------------------------------------------------------------
            // Reorder loop trims for UV sequential connectivity.
            // TopExp_Explorer does not guarantee wire-traversal order for
            // native OCCT shapes.  Sort the loop's trim list so that each
            // trim's UV end-point matches the next trim's UV start-point.
            // We skip this for loops that contain singular or seam trims,
            // because openNURBS is lenient about UV continuity at those
            // junctions and sorting might break valid arrangements created
            // by ON_BrepToOCCT.
            // ---------------------------------------------------------------
            {
                bool has_special = false;
                for (int k = 0; k < ol.m_ti.Count(); ++k) {
                    const ON_BrepTrim& t = brep.m_T[ol.m_ti[k]];
                    if (t.m_type == ON_BrepTrim::singular ||
                        t.m_type == ON_BrepTrim::seam) {
                        has_special = true;
                        break;
                    }
                }

                if (!has_special && ol.m_ti.Count() > 1) {
                    // Build UV start/end table for all trims in this loop
                    int n = ol.m_ti.Count();
                    std::vector<ON_2dPoint> uv_start(n), uv_end(n);
                    for (int k = 0; k < n; ++k) {
                        const ON_BrepTrim& t = brep.m_T[ol.m_ti[k]];
                        const ON_Curve* c2 = t.ProxyCurve();
                        if (c2) {
                            ON_Interval dom = t.ProxyCurveDomain();
                            ON_3dPoint s = c2->PointAt(dom.Min());
                            ON_3dPoint e = c2->PointAt(dom.Max());
                            uv_start[k] = ON_2dPoint(s.x, s.y);
                            uv_end[k]   = ON_2dPoint(e.x, e.y);
                        }
                    }

                    // Greedy chain sort: follow end→start connectivity
                    const double kUVTol = 1e-4;
                    std::vector<int> sorted;
                    std::vector<bool> used(n, false);
                    sorted.push_back(0);
                    used[0] = true;

                    for (int iter = 1; iter < n; ++iter) {
                        ON_2dPoint prev_end = uv_end[sorted.back()];
                        int best = -1;
                        double best_d = 1e30;
                        for (int j = 0; j < n; ++j) {
                            if (used[j]) continue;
                            double dx = uv_start[j].x - prev_end.x;
                            double dy = uv_start[j].y - prev_end.y;
                            double d  = std::sqrt(dx*dx + dy*dy);
                            if (d < best_d) { best_d = d; best = j; }
                        }
                        if (best >= 0) {
                            sorted.push_back(best);
                            used[best] = true;
                        }
                    }

                    // Apply the sorted order if it found all trims
                    if ((int)sorted.size() == n) {
                        ON_SimpleArray<int> new_ti(n);
                        for (int k = 0; k < n; ++k)
                            new_ti.Append(ol.m_ti[sorted[k]]);
                        for (int k = 0; k < n; ++k)
                            ol.m_ti[k] = new_ti[k];
                    }
                }
            }

            // Fix up loop type based on orientation
            ol.m_type = (ol.m_loop_index == of.m_li[0])
                            ? ON_BrepLoop::outer
                            : ON_BrepLoop::inner;
        }
    }

    brep.SetTolerancesBoxesAndFlags(/*bLazy=*/true);
    return brep.IsValid() ? true : false;
}

} // namespace open2open
