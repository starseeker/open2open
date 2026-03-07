# OpenNURBS to OpenCASCADE B-Rep Translation Plan - Summary

## Document Overview

This is a summary of the detailed analysis in `NURBS_BR_REP_ANALYSIS.md`. Refer to that document for complete specifications.

## Quick Reference: Key Differences

### 1. NURBS Curve Representation

**OpenNURBS (ON_NurbsCurve)**
- Uses imperative, pointer-based architecture
- Single flattened array for control points: `m_cv[]` with `m_cv_stride`
- Weights embedded in homogeneous coordinates (last value in each CP)
- Single knot array with implicit multiplicities
- Variable dimension support (m_dim)

**OCCT (Geom_BSplineCurve)**
- Uses OOP, handle-based architecture (Handle(T...))
- Separate `poles` (HArray1OfPnt) and `weights` (HArray1OfReal) arrays
- 1-indexed arrays (standard OCCT convention)
- Dual knot representation: unique `knots` + `mults`, plus `flatknots`
- Fixed 3D dimension (gp_Pnt)

**Translation Requirements:**
- Index remapping: 0-based → 1-based
- CV array restructuring: flattened → HArray1OfPnt
- Weight extraction: homogeneous → separate array
- Knot expansion: implicit → explicit multiplicities

---

### 2. NURBS Surface Representation

**OpenNURBS (ON_NurbsSurface)**
- Single flattened 2D grid: `m_cv[]` with `m_cv_stride[2]`
- Separate knot vectors for U and V: `m_knot[2]`
- Weights in homogeneous coordinates
- Variable dimension support

**OCCT (Geom_BSplineSurface)**
- 2D HArray: `poles` (HArray2OfPnt) with 1-indexed bounds
- Separate `weights` array (HArray2OfReal)
- Dual knot representation per direction (u/v)
- Fixed 3D dimension

**Translation Requirements:**
- Convert 2D flattened → HArray2OfPnt grid
- Extract weights to separate 2D array
- Expand knot multiplicities for both directions
- Handle parametric space orientation

---

### 3. B-Rep Topology Structure

**OpenNURBS Architecture**
- Single ON_Brep container with indexed arrays
- Flat topology: `m_V[]`, `m_E[]`, `m_T[]`, `m_L[]`, `m_F[]`
- Explicit cross-references: `edge.m_c3i`, `trim.m_c2i`, `trim.m_ei`
- Geometry in separate arrays: `m_C2[]`, `m_C3[]`, `m_S[]`
- Single curve per edge
- Orientation via `m_bRev` boolean flags
- Implicit location (in geometry coordinates)

**OCCT Architecture**
- Hierarchical: TopoDS_Shape (wrapper) → TopoDS_TShape (immutable)
- BRep_TEdge has `BRep_ListOfCurveRepresentation` (multiple representations)
- Orientation via `TopAbs_Orientation` enum (FORWARD/REVERSED/EXTERNAL)
- Explicit `TopLoc_Location` transformation per shape
- Surface stored in BRep_TFace

**Key Mapping:**
- `ON_Brep` → `TopoDS_Compound` or application-level container
- `m_V[]` (ON_BrepVertex) → `TopoDS_Vertex` + BRep_TVertex
- `m_E[]` (ON_BrepEdge) → `TopoDS_Edge` + BRep_TEdge with curve list
- `m_F[]` (ON_BrepFace) → `TopoDS_Face` + BRep_TFace with surface
- `m_C3[]` (3D curves) → BRep_Curve3D in edge representation list
- `m_C2[]` (2D curves) → BRep_CurveOnSurface in edge representation list
- `m_S[]` (surfaces) → Inside BRep_TFace

---

## Detailed Translation Steps

### Phase 1: NURBS Curves (Priority: HIGH)

```
Source: ON_NurbsCurve
Target: Geom_BSplineCurve

Steps:
1. Extract curve parameters
   - rational_flag = m_is_rat
   - degree = m_order - 1
   - num_poles = m_cv_count
   - periodic_flag = (inferred from knot values)

2. Convert control points
   For each CP[i]:
     - Extract from: m_cv[i*m_cv_stride + j] for j in [0, m_dim)
     - Create: gp_Pnt(x, y, z)
     - Store in: poles array (1-indexed)

3. Extract weights (if rational)
   For each CP[i]:
     - weight = m_cv[i*m_cv_stride + m_dim]
     - Store in: weights array (1-indexed)

4. Expand knot vector
   - Input: m_knot array (length = order + cv_count - 2)
   - Output: unique knots + multiplicities
   - Algorithm: Count consecutive knots
   - Create: HArray1OfReal knots, HArray1OfInteger mults

5. Construct Geom_BSplineCurve
   Constructor(poles, knots, mults, degree, periodic)

6. Validate
   - Check knot properties
   - Verify CP count vs knot count
   - Check continuity assumptions
```

**Code Template:**
```cpp
// Convert ON_NurbsCurve -> Geom_BSplineCurve
Handle(Geom_BSplineCurve) ConvertCurve(const ON_NurbsCurve& on_curve) {
  // 1. Extract parameters
  int degree = on_curve.Degree();
  int num_poles = on_curve.CVCount();
  bool is_rational = (on_curve.m_is_rat == 1);
  
  // 2. Create poles array (1-indexed)
  TColgp_Array1OfPnt poles(1, num_poles);
  for (int i = 0; i < num_poles; i++) {
    ON_3dPoint on_pt = on_curve.CV(i);  // Get euclidean point
    gp_Pnt pnt(on_pt.x, on_pt.y, on_pt.z);
    poles.SetValue(i+1, pnt);  // 1-indexed
  }
  
  // 3. Extract weights (if rational)
  TColStd_Array1OfReal weights(1, num_poles);
  if (is_rational) {
    for (int i = 0; i < num_poles; i++) {
      // Extract weight from homogeneous coordinate
      double w = on_curve.m_cv[i * on_curve.m_cv_stride + on_curve.m_dim];
      weights.SetValue(i+1, w);
    }
  }
  
  // 4. Expand knot vector
  int knot_count = degree + num_poles - 1;
  TColStd_Array1OfReal knots(1, knot_count);
  TColStd_Array1OfInteger mults(1, knot_count);
  // ... expand from on_curve.m_knot ...
  
  // 5. Create OCCT curve
  if (is_rational) {
    return new Geom_BSplineCurve(poles, weights, knots, mults, degree);
  } else {
    return new Geom_BSplineCurve(poles, knots, mults, degree);
  }
}
```

### Phase 2: NURBS Surfaces (Priority: HIGH)

```
Source: ON_NurbsSurface
Target: Geom_BSplineSurface

Steps: Similar to curves but in 2D
1. Extract U/V parameters
2. Convert 2D CP grid: m_cv (flattened) → poles (HArray2OfPnt)
3. Extract U/V weights (if rational)
4. Expand U and V knot vectors separately
5. Construct Geom_BSplineSurface
6. Validate
```

### Phase 3: B-Rep Topology (Priority: MEDIUM)

```
Source: ON_Brep + contained objects
Target: TopoDS_* hierarchy

Steps:
1. Create vertices
   For each ON_BrepVertex v:
     - point = v.m_position
     - tolerance = v.m_tolerance
     - Create: BRep_TVertex + TopoDS_Vertex wrapper

2. Create edges with curves
   For each ON_BrepEdge e:
     - Create: TopoDS_Edge wrapper
     - Attach: BRep_TEdge (from underlying TShape)
     - Add 3D curve: BRep_Curve3D from m_C3[e.m_c3i]
     - For each trim using this edge:
       Add: BRep_CurveOnSurface from m_C2[trim.m_c2i]
       Reference: Surface from m_S[face.m_si]

3. Create faces
   For each ON_BrepFace f:
     - Create: TopoDS_Face wrapper
     - Attach: BRep_TFace (from underlying TShape)
     - Set surface: m_S[f.m_si]
     - Add loops via edge wires

4. Create loops
   For each ON_BrepLoop l:
     - Create: TopoDS_Wire
     - Add edges from trims in order

5. Connect topology
   - Vertex locations
   - Edge endpoints (vi[0], vi[1])
   - Face orientation (m_bRev)
   - Trim types and ISO flags

6. Set tolerances
   - Vertex tolerance
   - Edge tolerance
   - Face tolerance
```

### Phase 4: Data Validation

```
For each translated object:
1. NURBS Curve
   ✓ Degree matches
   ✓ CP count matches
   ✓ Knot vector valid (non-decreasing, correct length)
   ✓ Weight values preserved (if rational)
   ✓ Domain matches

2. NURBS Surface
   ✓ U/V degrees match
   ✓ U/V CP counts match
   ✓ Knot vectors valid (both directions)
   ✓ Weight grid preserved (if rational)
   ✓ Parametric space correct

3. B-Rep Topology
   ✓ Vertex coordinates match
   ✓ Edge connectivity correct
   ✓ Face loops closed
   ✓ Orientation consistent
   ✓ Tolerance values preserved
```

---

## Index Conversion Reference

**For Arrays:**
```
OpenNURBS (0-based):  m_array[0], m_array[1], ..., m_array[n-1]
OCCT (1-based):       array.Value(1), array.Value(2), ..., array.Value(n)

Conversion in loop:
for (int on_idx = 0; on_idx < count; on_idx++) {
  int occt_idx = on_idx + 1;  // 0-based -> 1-based
  // ...
}
```

**For 2D Arrays:**
```
OpenNURBS (flattened):
  CV[i,j] = m_cv[i*stride[0] + j*stride[1]]

OCCT (2D indexed):
  CV[i,j] = poles->Value(i+1, j+1)  // Both 1-indexed
```

---

## API Quick Reference

### Creating OCCT Curves/Surfaces
```cpp
// Curve
Handle(Geom_BSplineCurve) crv = new Geom_BSplineCurve(
  poles,           // TColgp_Array1OfPnt (1-indexed)
  weights,         // TColStd_Array1OfReal (1-indexed, optional)
  knots,           // TColStd_Array1OfReal (unique values)
  mults,           // TColStd_Array1OfInteger (multiplicities)
  degree,          // Degree (not order)
  periodic         // bool
);

// Surface
Handle(Geom_BSplineSurface) srf = new Geom_BSplineSurface(
  poles,           // TColgp_Array2OfPnt (1-indexed)
  uweights,        // TColStd_Array2OfReal (optional)
  uknots,          // TColStd_Array1OfReal (unique)
  vknots,          // TColStd_Array1OfReal (unique)
  umults,          // TColStd_Array1OfInteger
  vmults,          // TColStd_Array1OfInteger
  udegree,         // Degree (not order)
  vdegree,         // Degree (not order)
  uperiodic,       // bool
  vperiodic        // bool
);
```

### Creating OCCT Topology
```cpp
// Vertex
BRep_Builder builder;
TopoDS_Vertex v;
builder.MakeVertex(v, gp_Pnt(x, y, z), tolerance);

// Edge
TopoDS_Edge e;
builder.MakeEdge(e, curve, tolerance);
builder.Add(e, v1);
builder.Add(e, v2);

// Face
TopoDS_Face f;
builder.MakeFace(f, surface, tolerance);

// Wire & Loop
TopoDS_Wire w;
builder.MakeWire(w);
for (each edge in loop) {
  builder.Add(w, edge);
}
builder.Add(f, w);  // Add wire to face
```

---

## Testing Checklist

- [ ] Unit tests for curve conversion
- [ ] Unit tests for surface conversion
- [ ] B-Rep topology tests
- [ ] Round-trip tests (OpenNURBS → OCCT → compare)
- [ ] Edge case tests (degenerate cases, singular surfaces)
- [ ] Tolerance validation
- [ ] Performance benchmarks
- [ ] Memory leak checks

---

## File References

| File | Purpose |
|------|---------|
| `NURBS_BR_REP_ANALYSIS.md` | Complete technical analysis |
| `/opennurbs-8.24.25281.15001/opennurbs_nurbscurve.h` | ON_NurbsCurve class |
| `/opennurbs-8.24.25281.15001/opennurbs_nurbssurface.h` | ON_NurbsSurface class |
| `/opennurbs-8.24.25281.15001/opennurbs_brep.h` | ON_Brep topology |
| `/OCCT-7_9_3/src/Geom/Geom_BSplineCurve.hxx` | OCCT B-Spline curve |
| `/OCCT-7_9_3/src/Geom/Geom_BSplineSurface.hxx` | OCCT B-Spline surface |
| `/OCCT-7_9_3/src/BRep/BRep_T*.hxx` | OCCT B-Rep topology |

---

## Next Steps

1. **Review** this summary and the detailed analysis document
2. **Implement** Phase 1 (curve translation) with full test coverage
3. **Implement** Phase 2 (surface translation)
4. **Implement** Phase 3 (topology, starting with vertices)
5. **Validate** each phase with test cases
6. **Document** any deviations from this plan
7. **Refine** based on real-world data testing

