# NURBS B-Rep Translation Library: opennurbs ↔ OpenCASCADE (OCCT)

## Overview

This document describes a plan for implementing a C++ library (`open2open`) that bridges
[openNURBS](https://github.com/mcneel/opennurbs) (the geometry kernel used by Rhino/BRL-CAD)
and [OpenCASCADE Technology](https://dev.opencascade.org/) (OCCT), enabling lossless
round-trip conversion of NURBS B-Rep solids between the two systems.

The primary focus is correct translation of NURBS-based B-Rep geometry and topology.  This
is the most critical component: without reliable B-Rep conversion everything else is
academic.  Other geometry types (meshes, annotations, etc.) are explicitly out of scope for
the initial implementation.

---

## 1. Source Code Baseline

| Item | Version | Key headers |
|------|---------|-------------|
| openNURBS | 8.24.25281.15001 | `opennurbs_brep.h`, `opennurbs_nurbscurve.h`, `opennurbs_nurbssurface.h`, `opennurbs_knot.h` |
| OCCT | 7.9.3 | `Geom/Geom_BSplineCurve.hxx`, `Geom/Geom_BSplineSurface.hxx`, `Geom2d/Geom2d_BSplineCurve.hxx`, `BRep/BRep_Builder.hxx`, `TopoDS/*.hxx` |

---

## 2. Key Structural Differences

Understanding the differences between the two representations is essential before any code
is written.

### 2.1 NURBS Curve Representation

| Aspect | openNURBS (`ON_NurbsCurve`) | OCCT (`Geom_BSplineCurve`) |
|--------|-----------------------------|----------------------------|
| Control point storage | Flat `double* m_cv` array with `m_cv_stride` | `Handle(TColgp_HArray1OfPnt)` — 1-indexed |
| Weight storage | Homogeneous coordinates: last element of each CV | Separate `Handle(TColStd_HArray1OfReal)` weights array |
| Knot vector | Single `double* m_knot` array (length = `m_order + m_cv_count - 2`); multiplicities are implicit (repeated values) | Unique `knots` array + `mults` array (plus pre-expanded `flatknots`) |
| Degree | `m_order - 1` | `deg` (direct) |
| Rationality flag | `m_is_rat` (int 0/1) | `rational` (Standard_Boolean) |
| Array indexing | 0-based | 1-based (OCCT standard) |
| Dimensionality | Variable `m_dim` (supports 2D and 3D) | Fixed 3D (`gp_Pnt`) |
| Periodicity detection | `ON_IsKnotVectorPeriodic()` helper | `periodic` flag stored directly |

**2D parameter-space curves:**  openNURBS stores 2D trim curves in `ON_Brep::m_C2[]` as
`ON_Curve*` (often `ON_NurbsCurve` with `m_dim=2`).  The OCCT equivalent is
`Geom2d_BSplineCurve`, stored in the edge's curve-representation list as
`BRep_CurveOnSurface`.

### 2.2 NURBS Surface Representation

| Aspect | openNURBS (`ON_NurbsSurface`) | OCCT (`Geom_BSplineSurface`) |
|--------|-------------------------------|------------------------------|
| Control point grid | Flat `double* m_cv` with `m_cv_stride[2]`; `CV(i,j) = m_cv + i*m_cv_stride[0] + j*m_cv_stride[1]` | `Handle(TColgp_HArray2OfPnt)` — 1-indexed in both dimensions |
| Weight grid | Homogeneous coordinate in each CV | `Handle(TColStd_HArray2OfReal)` |
| U/V knots | `m_knot[0]` and `m_knot[1]` (implicit multiplicities) | Separate unique + multiplicity arrays for each direction |
| U/V degrees | `m_order[0]-1`, `m_order[1]-1` | `udeg`, `vdeg` |
| CV counts | `m_cv_count[0]`, `m_cv_count[1]` | `NbUPoles()`, `NbVPoles()` |

### 2.3 B-Rep Topology Architecture

**openNURBS** uses a flat, array-based model inside a single `ON_Brep` container:

```
ON_Brep
├── Geometry pools
│   ├── m_C2[]  — 2D parameter-space curves (ON_Curve*)
│   ├── m_C3[]  — 3D edge curves (ON_Curve*)
│   └── m_S[]   — Surfaces (ON_Surface*)
└── Topology arrays (all cross-referenced by integer index)
    ├── m_V[]   — ON_BrepVertex  (point + tolerance; m_ei[] → adjacent edges)
    ├── m_E[]   — ON_BrepEdge    (m_c3i → 3D curve; m_vi[2] → vertices; m_ti[] → trims)
    ├── m_T[]   — ON_BrepTrim    (m_c2i → 2D curve; m_ei → edge; m_li → loop; m_bRev3d)
    ├── m_L[]   — ON_BrepLoop    (m_ti[] → ordered trims; m_fi → face; type = outer/inner/…)
    └── m_F[]   — ON_BrepFace    (m_si → surface; m_li[] → loops; m_bRev → orientation)
```

**OCCT** uses an immutable-shape / mutable-wrapper architecture:

```
TopoDS_Shape (wrapper: location + orientation)
└── TopoDS_TShape (shared, immutable)
    ├── BRep_TVertex  → gp_Pnt, tolerance
    ├── BRep_TEdge    → BRep_ListOfCurveRepresentation
    │   ├── BRep_Curve3D           (the 3D edge curve)
    │   ├── BRep_CurveOnSurface    (2D pcurve + surface reference)
    │   └── … (polygon, triangulation representations)
    ├── TopoDS_TWire  (ordered sequence of edges, = loop)
    ├── BRep_TFace    → Handle(Geom_Surface) + TopLoc_Location + tolerance
    └── TopoDS_TShell (set of faces)
```

| openNURBS concept | OCCT equivalent |
|-------------------|-----------------|
| `ON_Brep` | `TopoDS_Compound` (or `TopoDS_Solid`/`TopoDS_Shell`) |
| `ON_BrepVertex` | `TopoDS_Vertex` / `BRep_TVertex` |
| `ON_BrepEdge` | `TopoDS_Edge` / `BRep_TEdge` |
| `ON_BrepTrim` | pcurve entry in `BRep_CurveOnSurface` + edge orientation in `TopoDS_Edge` |
| `ON_BrepLoop` | `TopoDS_Wire` |
| `ON_BrepFace` | `TopoDS_Face` / `BRep_TFace` |
| `m_C3[]` (3D curve) | `BRep_Curve3D` inside edge representation list |
| `m_C2[]` (2D curve) | `Geom2d_Curve` inside `BRep_CurveOnSurface` |
| `m_S[]` (surface) | `Geom_Surface` inside `BRep_TFace` |
| `m_bRev` (face orientation) | `TopAbs_Orientation` on `TopoDS_Face` wrapper |

---

## 3. Library Design

### 3.1 Proposed API

The library will expose two primary translation functions (and their inverses):

```cpp
// opennurbs → OCCT
// Returns a TopoDS_Shape containing a Shell or Solid equivalent to the input ON_Brep.
TopoDS_Shape ON_BrepToOCCT(const ON_Brep& brep, double linear_tolerance = 1e-6);

// OCCT → opennurbs
// Populates an ON_Brep from any TopoDS_Shape that contains Shell/Face topology.
bool OCCTToON_Brep(const TopoDS_Shape& shape, ON_Brep& brep, double linear_tolerance = 1e-6);
```

Lower-level helpers will also be public so callers can translate individual geometric
primitives independently:

```cpp
// Geometry helpers
Handle(Geom_BSplineCurve)    ON_NurbsCurveToOCCT(const ON_NurbsCurve& c);
Handle(Geom_BSplineSurface)  ON_NurbsSurfaceToOCCT(const ON_NurbsSurface& s);
Handle(Geom2d_BSplineCurve)  ON_NurbsCurve2dToOCCT(const ON_NurbsCurve& c); // m_dim==2

bool OCCTCurveToON(const Handle(Geom_BSplineCurve)& c, ON_NurbsCurve& out);
bool OCCTSurfaceToON(const Handle(Geom_BSplineSurface)& s, ON_NurbsSurface& out);
```

### 3.2 File Layout

```
open2open/
├── include/
│   └── open2open/
│       ├── open2open.h          # Main public header
│       ├── geom_convert.h       # Curve / surface helpers
│       └── brep_convert.h       # B-Rep topology helpers
├── src/
│   ├── geom_convert.cpp
│   └── brep_convert.cpp
├── tests/
│   ├── test_curves.cpp
│   ├── test_surfaces.cpp
│   └── test_brep.cpp
└── CMakeLists.txt
```

---

## 4. Implementation Plan

Work is divided into four phases.  Each phase must be fully tested before the next begins.

---

### Phase 1: NURBS Curve Translation

**Priority: Critical** — curves are the building block for everything else.

#### 4.1.1 `ON_NurbsCurve` → `Geom_BSplineCurve`

```
1. Read openNURBS parameters
   degree     = on_curve.m_order - 1
   num_poles  = on_curve.m_cv_count
   is_rational = (on_curve.m_is_rat != 0)
   is_periodic = ON_IsKnotVectorPeriodic(on_curve.m_knot, on_curve.m_order,
                                         on_curve.m_cv_count)

2. Extract 3D control points (poles)
   for i in [0, num_poles):
     ptr = on_curve.m_cv + i * on_curve.m_cv_stride
     poles[i+1] = gp_Pnt(ptr[0], ptr[1], ptr[2])   // 1-indexed target

3. Extract weights (only if rational)
   for i in [0, num_poles):
     ptr = on_curve.m_cv + i * on_curve.m_cv_stride
     w = ptr[on_curve.m_dim]   // weight is last homogeneous coordinate
     // OCCT expects Euclidean poles, not homogeneous; divide xyz by w
     poles[i+1] = gp_Pnt(ptr[0]/w, ptr[1]/w, ptr[2]/w)
     weights[i+1] = w

4. Convert knot vector (implicit multiplicities → explicit)
   The openNURBS knot array has length (m_order + m_cv_count - 2).
   Scan left-to-right, counting consecutive equal values:
     unique_knots[] ← distinct values
     mults[]        ← count of each distinct value
   Note: openNURBS clamped curves have (degree) repeated knots at each end,
   i.e., mults[0] = mults[last] = degree+1.  Periodic curves have
   mults[0] = mults[last] = 1 (or lower).

5. Construct Geom_BSplineCurve
   if (is_rational)
     return new Geom_BSplineCurve(poles, weights, knots, mults, degree, is_periodic);
   else
     return new Geom_BSplineCurve(poles, knots, mults, degree, is_periodic);
```

**Dimension check:** if `on_curve.m_dim == 2` the curve is a 2D parameter-space trim
curve; route it to `Geom2d_BSplineCurve` instead (same algorithm, use `gp_Pnt2d`).

#### 4.1.2 `Geom_BSplineCurve` → `ON_NurbsCurve`

```
1. Read OCCT parameters
   degree      = curve->Degree()
   num_poles   = curve->NbPoles()
   is_rational = curve->IsRational()
   is_periodic = curve->IsPeriodic()

2. Create ON_NurbsCurve
   on_curve.Create(3, is_rational, degree+1, num_poles)

3. Set control points (Euclidean)
   for i in [1, num_poles]:
     w  = is_rational ? curve->Weight(i) : 1.0
     p  = curve->Pole(i)
     // openNURBS rational curves store homogeneous coordinates
     on_curve.SetCV(i-1, ON_4dPoint(p.X()*w, p.Y()*w, p.Z()*w, w))

4. Expand OCCT knot vector to flat openNURBS form
   for each unique knot k at index j (1-indexed):
     repeat curve->Multiplicity(j) times → append k to flat_knots[]
   // flat_knots length must equal degree + num_poles - 1
   // (openNURBS uses superfluous-knot convention: no outer repetition beyond degree)
   for (int ki = 0; ki < flat_knots.size(); ki++)
     on_curve.m_knot[ki] = flat_knots[ki]
```

---

### Phase 2: NURBS Surface Translation

**Priority: Critical.**

#### 4.2.1 `ON_NurbsSurface` → `Geom_BSplineSurface`

```
1. Read parameters
   udeg  = on_srf.m_order[0] - 1
   vdeg  = on_srf.m_order[1] - 1
   nu    = on_srf.m_cv_count[0]
   nv    = on_srf.m_cv_count[1]
   is_rational = (on_srf.m_is_rat != 0)

2. Extract 2D control point grid
   poles (HArray2OfPnt, bounds [1..nu][1..nv])
   for i in [0, nu):
     for j in [0, nv):
       ptr = on_srf.m_cv + i*on_srf.m_cv_stride[0] + j*on_srf.m_cv_stride[1]
       if rational: w = ptr[on_srf.m_dim]; poles[i+1][j+1] = Euclidean(ptr)/w
       else: poles[i+1][j+1] = gp_Pnt(ptr[0], ptr[1], ptr[2])

3. Extract weight grid (if rational)
   weights (HArray2OfReal, same bounds)

4. Expand U and V knot vectors independently (same algorithm as curves)

5. Construct Geom_BSplineSurface
   if (is_rational)
     return new Geom_BSplineSurface(poles, weights,
                                    uknots, vknots, umults, vmults,
                                    udeg, vdeg,
                                    u_periodic, v_periodic);
   else
     return new Geom_BSplineSurface(poles,
                                    uknots, vknots, umults, vmults,
                                    udeg, vdeg,
                                    u_periodic, v_periodic);
```

#### 4.2.2 `Geom_BSplineSurface` → `ON_NurbsSurface`

The inverse follows the same logic as the curve inverse: read OCCT degree/poles/weights/
knots, create the `ON_NurbsSurface` with `Create()`, populate homogeneous control points,
and expand the flat knot vectors.

---

### Phase 3: B-Rep Topology Translation

**Priority: High** — depends on Phases 1 and 2.

This is the most complex phase because openNURBS and OCCT have fundamentally different
topology architectures.  The mapping must maintain the correct association of 3D curves,
2D parameter-space curves, surfaces, vertices, and orientation flags.

#### 4.3.1 `ON_Brep` → OCCT topology

**Step 1 — Translate geometry pools**

```cpp
// 3D curves
std::vector<Handle(Geom_Curve)> c3_map(brep.m_C3.Count());
for (int i = 0; i < brep.m_C3.Count(); i++) {
    const ON_NurbsCurve* nc = ON_NurbsCurve::Cast(brep.m_C3[i]);
    c3_map[i] = nc ? ON_NurbsCurveToOCCT(*nc) : nullptr;
}

// 2D trim curves
std::vector<Handle(Geom2d_Curve)> c2_map(brep.m_C2.Count());
for (int i = 0; i < brep.m_C2.Count(); i++) {
    const ON_NurbsCurve* nc = ON_NurbsCurve::Cast(brep.m_C2[i]);
    c2_map[i] = nc ? ON_NurbsCurve2dToOCCT(*nc) : nullptr;
}

// Surfaces
std::vector<Handle(Geom_Surface)> s_map(brep.m_S.Count());
for (int i = 0; i < brep.m_S.Count(); i++) {
    const ON_NurbsSurface* ns = ON_NurbsSurface::Cast(brep.m_S[i]);
    s_map[i] = ns ? ON_NurbsSurfaceToOCCT(*ns) : nullptr;
}
```

Non-NURBS geometry types (e.g., `ON_RevSurface`, `ON_PlaneSurface`, `ON_ArcCurve`) must be
converted to NURBS first via `ON_Curve::NurbsCurve()` / `ON_Surface::NurbsSurface()` before
translation.  This produces slightly larger representations but guarantees coverage.

**Step 2 — Create vertices**

```cpp
BRep_Builder B;
std::vector<TopoDS_Vertex> v_map(brep.m_V.Count());
for (int i = 0; i < brep.m_V.Count(); i++) {
    const ON_BrepVertex& v = brep.m_V[i];
    double tol = (v.m_tolerance == ON_UNSET_VALUE) ? linear_tolerance
                                                   : v.m_tolerance;
    B.MakeVertex(v_map[i],
                 gp_Pnt(v.Point().x, v.Point().y, v.Point().z),
                 tol);
}
```

**Step 3 — Create edges**

Each `ON_BrepEdge` maps to one `TopoDS_Edge`.  The edge carries:
- Its 3D curve (`m_c3i` → `c3_map[e.m_c3i]`)
- Its parameter range (derived from the 3D curve's domain)
- Its start/end vertices (`m_vi[0]`, `m_vi[1]`)
- Its tolerance (`m_tolerance`)

The 2D pcurves are attached later, per face, when the faces are constructed.

```cpp
std::vector<TopoDS_Edge> e_map(brep.m_E.Count());
for (int i = 0; i < brep.m_E.Count(); i++) {
    const ON_BrepEdge& e = brep.m_E[i];
    B.MakeEdge(e_map[i]);
    if (e.m_c3i >= 0 && c3_map[e.m_c3i]) {
        double t0 = e.Domain().Min(), t1 = e.Domain().Max();
        B.UpdateEdge(e_map[i], c3_map[e.m_c3i], linear_tolerance);
        B.Range(e_map[i], t0, t1, /*only3d=*/true);
    }
    if (e.m_vi[0] >= 0) B.Add(e_map[i], v_map[e.m_vi[0]]);
    if (e.m_vi[1] >= 0) B.Add(e_map[i], v_map[e.m_vi[1]]);
    if (e.m_tolerance != ON_UNSET_VALUE)
        B.UpdateEdge(e_map[i], e.m_tolerance);
}
```

**Step 4 — Create faces, add pcurves, and build wires**

This is the most intricate step.  For each face:

1. Create a `TopoDS_Face` with the underlying surface.
2. Honour the `m_bRev` flag by setting `TopAbs_REVERSED` orientation on the face.
3. For each loop on the face, create a `TopoDS_Wire` by iterating its trims in order.
4. For each trim, add the corresponding edge (with correct orientation derived from
   `ON_BrepTrim::m_bRev3d`) to the wire, and attach the 2D pcurve to that edge/face pair.

```cpp
std::vector<TopoDS_Face> f_map(brep.m_F.Count());
for (int fi = 0; fi < brep.m_F.Count(); fi++) {
    const ON_BrepFace& f = brep.m_F[fi];
    B.MakeFace(f_map[fi], s_map[f.m_si], linear_tolerance);

    // Apply face reversal
    if (f.m_bRev)
        f_map[fi].Reverse();

    // Loops → Wires
    for (int li_idx = 0; li_idx < f.m_li.Count(); li_idx++) {
        const ON_BrepLoop& loop = brep.m_L[f.m_li[li_idx]];
        TopoDS_Wire wire;
        B.MakeWire(wire);

        for (int ti_idx = 0; ti_idx < loop.m_ti.Count(); ti_idx++) {
            const ON_BrepTrim& trim = brep.m_T[loop.m_ti[ti_idx]];
            int ei = trim.m_ei;
            if (ei < 0) continue; // singular / degenerate trim

            // Attach 2D pcurve to this edge on this face
            if (trim.m_c2i >= 0 && c2_map[trim.m_c2i]) {
                B.UpdateEdge(e_map[ei], c2_map[trim.m_c2i],
                             f_map[fi], trim.m_tolerance[0]);
                B.Range(e_map[ei], f_map[fi],
                        trim.Domain().Min(), trim.Domain().Max());
            }

            // Edge orientation: trim.m_bRev3d means the trim's direction
            // is opposite to the edge's 3D curve direction
            TopoDS_Edge oriented_edge = e_map[ei];
            if (trim.m_bRev3d)
                oriented_edge.Reverse();
            B.Add(wire, oriented_edge);
        }
        B.Add(f_map[fi], wire);
    }
}
```

**Step 5 — Assemble shell / solid**

```cpp
TopoDS_Shell shell;
B.MakeShell(shell);
for (int fi = 0; fi < brep.m_F.Count(); fi++)
    B.Add(shell, f_map[fi]);
B.MakeSolid(result_solid);
B.Add(result_solid, shell);
```

If `ON_Brep::IsValid()` is true and the result passes `BRepCheck_Analyzer`, the solid will
be a valid closed manifold.

#### 4.3.2 OCCT topology → `ON_Brep`

The inverse traversal uses OCCT's topological iterators:

```
TopExp_Explorer (TopoDS_Shape, TopAbs_FACE) → faces
  BRep_Tool::Surface(face) → Geom_Surface
  TopExp_Explorer (face, TopAbs_WIRE) → wires = loops
    TopExp_Explorer (wire, TopAbs_EDGE) → edges
      BRep_Tool::Curve(edge, first, last) → Geom_Curve
      BRep_Tool::CurveOnSurface(edge, face, first, last) → Geom2d_Curve (pcurve)
      BRep_Tool::Pnt(vertex) → ON_3dPoint
```

The mapping from `TopAbs_Orientation` back to openNURBS `m_bRev` / `m_bRev3d` is:
- Face orientation `REVERSED` → set `ON_BrepFace::m_bRev = true`
- Edge orientation `REVERSED` within a wire → set `ON_BrepTrim::m_bRev3d = true`

Geometry is converted using the inverse functions from Phases 1 and 2.

---

### Phase 4: Validation and Round-trip Testing

A translation is only trustworthy if it can be verified.

#### Geometric validation

After translating a curve or surface, sample both the original and the translated object at
a grid of parameter values and verify that point distances are below the specified
tolerance.  The openNURBS `ON_Curve::Evaluate()` / `ON_Surface::Evaluate()` methods and
OCCT `Geom_Curve::D0()` / `Geom_Surface::D0()` are used for sampling.

#### Topology validation

- openNURBS: `ON_Brep::IsValid()` must return `true` both before and after a round-trip.
- OCCT: `BRepCheck_Analyzer(shape).IsValid()` must return `true` after translation from
  openNURBS, and `ShapeAnalysis_ShapeContents` can report global statistics.

#### Round-trip test protocol

```
1. Load a 3DM file with ON_BinaryArchive (openNURBS).
2. Translate ON_Brep → TopoDS_Shape (Phase 3, ON→OCCT direction).
3. Validate OCCT shape with BRepCheck_Analyzer.
4. Translate TopoDS_Shape → ON_Brep (Phase 3, OCCT→ON direction).
5. Validate result with ON_Brep::IsValid().
6. For each face pair (original vs. round-trip):
   a. Sample surface at (u,v) grid → compare point arrays.
   b. For each edge: sample 3D curve → compare.
   c. For each trim: sample 2D pcurve → compare parameter-space points.
7. Assert max deviation < linear_tolerance.
```

---

## 5. Critical Implementation Notes

### 5.1 openNURBS knot vector convention

openNURBS uses a **clamped** knot vector of length `m_order + m_cv_count - 2` (no outer
padding).  OCCT requires the full clamped knot vector with degree+1 repetitions at the
ends, expressed as `knots[]` + `mults[]`.  The conversion must:

1. **Scan** the flat openNURBS array left-to-right, collecting `(value, count)` pairs.
2. **Verify** that the first and last multiplicities are `== degree+1` for a clamped
   (non-periodic) curve; if not the curve is periodic or semi-clamped and must be handled
   accordingly.

The `ON_KnotMultiplicity()` and `ON_IsKnotVectorPeriodic()` helpers in `opennurbs_knot.h`
are useful here.

### 5.2 Homogeneous vs. Euclidean control points

openNURBS stores rational CVs in **homogeneous** form: `[X*w, Y*w, Z*w, w]`.  OCCT
`Geom_BSplineCurve` and `Geom_BSplineSurface` store poles in **Euclidean** form plus a
separate weight array.  On conversion from openNURBS to OCCT, divide XYZ by W before
storing in the poles array.  On conversion from OCCT to openNURBS, multiply XYZ by W
before storing in `m_cv`.

### 5.3 Non-NURBS geometry fallback

Both libraries support primitive surface types (planes, cylinders, cones, spheres, tori)
and analytic curves (lines, arcs, circles, ellipses) in addition to free-form NURBS.
The translation library must handle these in one of two ways:

- **Preferred**: Detect the type on the source side and construct the equivalent analytic
  type on the target side (e.g., `ON_PlaneSurface` → `Geom_Plane`).  This preserves
  exactness and avoids NURBS bloat.
- **Fallback**: Convert any non-NURBS geometry to NURBS via `ON_Surface::NurbsSurface()` /
  `GeomConvert::CurveToBSplineCurve()` before translating.

The initial implementation may use the fallback exclusively and add analytic-type
preservation as a follow-on optimization.

### 5.4 openNURBS `ON_BrepTrim` type mapping

openNURBS classifies trim curves into `boundary`, `mating`, `seam`, `singular`,
`crvonsrf`, and `ptonsrf` types.  OCCT does not have a direct equivalent.  The semantics
must be preserved through correct topology construction:

- `singular` trim: degenerate edge (a single point on the surface); use
  `BRep_Builder::Degenerated()`.
- `seam` trim: an edge on a closed surface where both sides of the seam share the same
  underlying 3D curve; attach two pcurves to the single edge using the two-argument
  `UpdateEdge()` overload.
- `ptonsrf`: a point (vertex) embedded on a face with no edge; represent as an isolated
  vertex inside a face using `BRep_Builder::UpdateVertex()`.

### 5.5 Tolerances

openNURBS carries per-vertex (`m_tolerance`), per-edge (`m_tolerance`), and per-trim
(`m_tolerance[2]`) tolerances.  OCCT enforces a stricter ordering: face tolerance ≤ edge
tolerance ≤ vertex tolerance.  The translation must promote tolerances upward as needed
to satisfy OCCT's invariant.

### 5.6 Orientation conventions

openNURBS face orientation is stored as `m_bRev` on `ON_BrepFace`.  In OCCT it is the
`TopAbs_Orientation` value on the `TopoDS_Face` wrapper (`FORWARD` = normal,
`REVERSED` = flipped).  Trim orientation is `m_bRev3d` on `ON_BrepTrim`.  In OCCT it
is the orientation of the `TopoDS_Edge` within the `TopoDS_Wire`.

---

## 6. Build System

The library will use CMake, linking against both opennurbs (as a static library) and OCCT
(via its standard `find_package(OpenCASCADE)` CMake support).

```cmake
cmake_minimum_required(VERSION 3.16)
project(open2open VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCASCADE REQUIRED COMPONENTS FoundationClasses ModelingAlgorithms)

add_library(open2open STATIC
    src/geom_convert.cpp
    src/brep_convert.cpp
)
target_include_directories(open2open
    PUBLIC  include
    PRIVATE ${OpenCASCADE_INCLUDE_DIR}
)
target_link_libraries(open2open
    PRIVATE opennurbs
            ${OpenCASCADE_FoundationClasses_LIBRARIES}
            ${OpenCASCADE_ModelingAlgorithms_LIBRARIES}
)

# Tests
enable_testing()
add_subdirectory(tests)
```

---

## 7. Testing Strategy

Tests are written with the source files already present in this repository.

| Test suite | What it exercises |
|------------|-------------------|
| `test_curves` | Round-trip `ON_NurbsCurve` → `Geom_BSplineCurve` → `ON_NurbsCurve` for rational and non-rational cubic, quintic, and uniform B-splines; verify degree, CV count, knot vectors, and point samples |
| `test_surfaces` | Round-trip `ON_NurbsSurface` → `Geom_BSplineSurface` → `ON_NurbsSurface`; bicubic, rational patches; U-periodic cylinder; verify gridded point samples |
| `test_brep` | Translate the example B-Rep models from `opennurbs-8.24.25281.15001/example_brep/` to OCCT and back; check `BRepCheck_Analyzer` validity and `ON_Brep::IsValid()` after the round-trip |
| `test_edge_cases` | Degenerate edges (poles), seam edges on tori, singular trims on cones, open vs. closed shells |

---

## 8. Phased Delivery

| Phase | Deliverable | Success criterion |
|-------|-------------|-------------------|
| 1 | `ON_NurbsCurveToOCCT` + inverse | Geometric round-trip within 1e-10 for standard test cases |
| 2 | `ON_NurbsSurfaceToOCCT` + inverse | Geometric round-trip within 1e-10 for standard test cases |
| 3 | `ON_BrepToOCCT` + inverse | `BRepCheck_Analyzer` and `ON_Brep::IsValid()` both pass after round-trip on all example files |
| 4 | Analytic type preservation (planes, cylinders, spheres) | No NURBS approximation for exact primitives |
| 5 | Performance and robustness | Handles all models in `example_brep/` under 1 second each; no memory leaks (Valgrind clean) |

---

## 9. References

- openNURBS 8.24 source:
  - `opennurbs_brep.h` — `ON_Brep`, `ON_BrepFace`, `ON_BrepEdge`, `ON_BrepTrim`, `ON_BrepLoop`, `ON_BrepVertex`
  - `opennurbs_nurbscurve.h` — `ON_NurbsCurve`
  - `opennurbs_nurbssurface.h` — `ON_NurbsSurface`
  - `opennurbs_knot.h` — `ON_KnotMultiplicity`, `ON_IsKnotVectorPeriodic`
- OCCT 7.9.3 source:
  - `Geom/Geom_BSplineCurve.hxx` — 3D B-spline curve
  - `Geom/Geom_BSplineSurface.hxx` — 3D B-spline surface
  - `Geom2d/Geom2d_BSplineCurve.hxx` — 2D (parameter-space) B-spline curve
  - `BRep/BRep_Builder.hxx` — topology construction API
  - `TopoDS/TopoDS_*.hxx` — shape wrapper classes
  - `TopExp/TopExp_Explorer.hxx` — topological traversal
  - `BRepCheck/BRepCheck_Analyzer.hxx` — validity checker
