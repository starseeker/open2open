# NURBS B-Rep Data Representation Analysis: OpenNURBS vs OpenCASCADE

## Executive Summary

This document provides a detailed comparison of how two leading open-source geometry libraries represent NURBS curves, surfaces, and B-Rep (Boundary Representation) topology structures. The analysis is based on the header files from OpenNURBS 8.24 and OpenCASCADE (OCCT) 7.9.3.

---

## 1. NURBS CURVE REPRESENTATION

### 1.1 OpenNURBS (ON_NurbsCurve)

**Location**: `opennurbs_nurbscurve.h`

**Class Definition**: `class ON_NurbsCurve : public ON_Curve`

**Key Member Variables**:
```cpp
int     m_dim;                          // Dimensionality (>=1)
int     m_is_rat;                       // 1=rational, 0=non-rational
int     m_order;                        // Order = degree + 1 (>=2)
int     m_cv_count;                     // Number of control vertices (>=order)
double* m_knot;                         // Knot vector (length = m_order + m_cv_count - 2)
int     m_cv_stride;                    // Stride between consecutive control vertices
double* m_cv;                           // Control point array
int     m_cv_capacity;                  // Allocated capacity for CV array
unsigned int m_knot_capacity_and_tags;  // Knot capacity + internal flags
```

**Storage Details**:
- **Control Points**: Stored in flattened array `m_cv` with stride `m_cv_stride`
  - Non-rational: `CV[i] = m_cv + i*m_cv_stride` → `[x, y, z, ...]` (m_dim values)
  - Rational: `CV[i] = m_cv + i*m_cv_stride` → `[x*w, y*w, z*w, ..., w]` (m_dim+1 homogeneous values)
  
- **Knot Vector**: Simple double array with length `m_order + m_cv_count - 2`
  - Interior knots: `m_knot[m_order-2]` to `m_knot[m_cv_count-1]`
  - Values must be non-decreasing

- **Weights**: Implicit in homogeneous coordinates for rational curves
  - Extract weight: `w = CV[i][m_dim]` (last coordinate)
  - Euclidean point: `(x/w, y/w, z/w, ...)`

**Key API Methods**:
```cpp
bool Create(int dimension, bool bIsRational, int order, int cv_count);
bool CreateClampedUniformNurbs(...);
bool CreatePeriodicUniformNurbs(...);

// Access
int Degree() const;                  // Returns m_order - 1
int SpanCount() const;
bool GetSpanVector(double* knot_values) const;
ON_Interval Domain() const;

// Modification
bool SetDomain(double t0, double t1);
bool ChangeClosedCurveSeam(double t);
```

---

### 1.2 OpenCASCADE (Geom_BSplineCurve)

**Location**: `Geom/Geom_BSplineCurve.hxx`

**Class Definition**: `class Geom_BSplineCurve : public Geom_BoundedCurve`

**Key Member Variables**:
```cpp
Standard_Boolean                 rational;      // true=rational, false=non-rational
Standard_Boolean                 periodic;      // true=periodic
GeomAbs_BSplKnotDistribution     knotSet;       // Knot distribution type
GeomAbs_Shape                    smooth;        // Smoothness level
Standard_Integer                 deg;           // Degree
Handle(TColgp_HArray1OfPnt)      poles;         // Control points (handles 1-indexed array)
Handle(TColStd_HArray1OfReal)    weights;       // Weights (handle, only if rational)
Handle(TColStd_HArray1OfReal)    flatknots;    // Flattened knot vector with multiplicities
Handle(TColStd_HArray1OfReal)    knots;         // Unique knot values
Handle(TColStd_HArray1OfInteger) mults;         // Knot multiplicities
Standard_Real                    maxderivinv;   // Cache for max derivative inverse
Standard_Boolean                 maxderivinvok; // Cache validity flag
```

**Storage Details**:
- **Control Points**: Handle to `TColgp_HArray1OfPnt` (1-indexed array)
  - Direct storage of `gp_Pnt` objects (3D points only)
  - Access: `poles->Value(1) to poles->Value(NbPoles())`
  
- **Weights**: Handle to `TColStd_HArray1OfReal` (only if rational)
  - 1-indexed array with same bounds as poles
  - Access: `weights->Value(index)` or `Weight(index)`

- **Knot Vector**: Dual representation
  - `knots`: Unique knot values (1-indexed)
  - `mults`: Multiplicities for each knot (1-indexed)
  - `flatknots`: Pre-computed flattened vector with multiplicities expanded
  - Flattened knot vector length = `NbKnots() + Degree` for non-periodic curves

**Construction**:
```cpp
Geom_BSplineCurve(
  const TColgp_Array1OfPnt& Poles,
  const TColStd_Array1OfReal& Knots,
  const TColStd_Array1OfInteger& Multiplicities,
  const Standard_Integer Degree,
  const Standard_Boolean Periodic = Standard_False
);

// Rational version:
Geom_BSplineCurve(
  const TColgp_Array1OfPnt& Poles,
  const TColStd_Array1OfReal& Weights,
  const TColStd_Array1OfReal& Knots,
  const TColStd_Array1OfInteger& Multiplicities,
  const Standard_Integer Degree,
  const Standard_Boolean Periodic = Standard_False
);
```

---

## 2. NURBS SURFACE REPRESENTATION

### 2.1 OpenNURBS (ON_NurbsSurface)

**Location**: `opennurbs_nurbssurface.h`

**Key Member Variables**:
```cpp
int     m_dim;                  // Dimensionality (>=1)
int     m_is_rat;               // 1=rational, 0=non-rational
int     m_order[2];             // Order = degree + 1 for U,V directions (>=2)
int     m_cv_count[2];          // Number of control vertices in U,V (>=order)
int     m_knot_capacity[2];     // Allocated capacity for knot arrays
double* m_knot[2];              // Knot vectors for U,V directions
int     m_cv_stride[2];         // Stride within each row/column
int     m_cv_capacity;          // Total allocated capacity for CV array
double* m_cv;                   // Control point grid (flattened)
```

**Storage Details**:
- **Control Point Grid**: Flattened 2D array
  - Access: `CV[i,j] = m_cv + (i*m_cv_stride[0] + j*m_cv_stride[1])`
  - Non-rational: `[x, y, z, ...]` per point (m_dim values)
  - Rational: `[x*w, y*w, z*w, ..., w]` per point (m_dim+1 homogeneous values)
  - Grid dimensions: `m_cv_count[0]` × `m_cv_count[1]`

- **Knot Vectors**: Two separate arrays
  - U: `m_knot[0]` length = `m_order[0] + m_cv_count[0] - 2`
  - V: `m_knot[1]` length = `m_order[1] + m_cv_count[1] - 2`

### 2.2 OpenCASCADE (Geom_BSplineSurface)

**Location**: `Geom/Geom_BSplineSurface.hxx`

**Key Member Variables**:
```cpp
Standard_Integer                 udeg;          // U degree
Standard_Integer                 vdeg;          // V degree
Handle(TColgp_HArray2OfPnt)      poles;         // Control points (2D array, 1-indexed)
Handle(TColStd_HArray2OfReal)    weights;       // Weights (2D array, only if rational)
Handle(TColStd_HArray1OfReal)    ufknots;       // U flattened knot vector
Handle(TColStd_HArray1OfReal)    vfknots;       // V flattened knot vector
Handle(TColStd_HArray1OfReal)    uknots;        // U unique knot values
Handle(TColStd_HArray1OfReal)    vknots;        // V unique knot values
Handle(TColStd_HArray1OfInteger) umults;        // U knot multiplicities
Handle(TColStd_HArray1OfInteger) vmults;        // V knot multiplicities
```

**Storage Details**:
- **Control Point Grid**: `TColgp_HArray2OfPnt` (1-indexed 2D array)
  - Bounds: `[1..NbUPoles] × [1..NbVPoles]`
  - Access: `poles->Value(uindex, vindex)`
  - 3D points only

- **Weights**: `TColStd_HArray2OfReal` (if rational)
  - Same 2D bounds as poles

- **Knot Vectors**: Dual representation per direction
  - U: `uknots` (unique), `umults` (multiplicities), `ufknots` (flattened)
  - V: `vknots` (unique), `vmults` (multiplicities), `vfknots` (flattened)

---

## 3. B-REP TOPOLOGY STRUCTURE

### 3.1 OpenNURBS Structure Overview

```
ON_Brep (main container)
├── Geometry Arrays
│   ├── m_C2[]   - 2D parameter space curves (ON_Curve*)
│   ├── m_C3[]   - 3D edge curves (ON_Curve*)
│   └── m_S[]    - Surfaces (ON_Surface*)
└── Topology Arrays
    ├── m_V[]    - Vertices (ON_BrepVertex)
    ├── m_E[]    - Edges (ON_BrepEdge)
    ├── m_T[]    - Trims (ON_BrepTrim)
    ├── m_L[]    - Loops (ON_BrepLoop)
    └── m_F[]    - Faces (ON_BrepFace)
```

#### 3.1.1 ON_BrepVertex

```cpp
class ON_BrepVertex : public ON_Point {
  ON_3dPoint point;           // Vertex location
  int m_vertex_index;         // Index in ON_Brep.m_V[]
  ON_SimpleArray<int> m_ei;   // Indices of edges at this vertex
  double m_tolerance;         // Vertex tolerance
};
```

#### 3.1.2 ON_BrepEdge

```cpp
class ON_BrepEdge : public ON_CurveProxy {
  int m_c3i;                  // Index in ON_Brep.m_C3[]
  int m_vi[2];                // Indices of start/end vertices
  ON_SimpleArray<int> m_ti;   // Indices of trims using this edge
  double m_tolerance;         // Edge tolerance
  bool m_bIsDegenerate;
  int m_edge_index;           // Index in ON_Brep.m_E[]
};
```

#### 3.1.3 ON_BrepTrim

```cpp
class ON_BrepTrim : public ON_CurveProxy {
  int m_c2i;                      // Index in ON_Brep.m_C2[] (2D trim curve)
  int m_ei;                       // Index in ON_Brep.m_E[] (3D edge)
  int m_vi[2];                    // Start/end vertices
  bool m_bRev3d;                  // Orientation flag
  enum TYPE {unknown, boundary, mating, seam, singular, crvonsrf, ptonsrf};
  TYPE m_type;
  ON_Surface::ISO m_iso;          // Isoparametric type
  int m_li;                       // Index of containing loop
  double m_tolerance[2];          // [u_tol, v_tol]
  int m_trim_index;               // Index in ON_Brep.m_T[]
};
```

#### 3.1.4 ON_BrepLoop

```cpp
class ON_BrepLoop : public ON_Geometry {
  ON_SimpleArray<int> m_ti;   // Trim indices in this loop
  enum TYPE {unknown, outer, inner, slit, crvonsrf, ptonsrf};
  TYPE m_type;
  int m_fi;                   // Index of containing face
  int m_loop_index;           // Index in ON_Brep.m_L[]
};
```

#### 3.1.5 ON_BrepFace

```cpp
class ON_BrepFace : public ON_SurfaceProxy {
  int m_si;                       // Index in ON_Brep.m_S[]
  ON_SimpleArray<int> m_li;       // Loop indices in this face
  int m_face_index;               // Index in ON_Brep.m_F[]
  bool m_bRev;                    // Surface orientation flag
  double m_tolerance;
};
```

### 3.2 OpenCASCADE B-Rep Structure

```
TopoDS_Shape (wrapper with location and orientation)
└── TopoDS_TShape (handle to immutable topology)
    ├── TopoDS_TVertex → BRep_TVertex (gp_Pnt, tolerance)
    ├── TopoDS_TEdge → BRep_TEdge
    │   ├── BRep_ListOfCurveRepresentation
    │   │   ├── BRep_Curve3D (3D curve)
    │   │   ├── BRep_CurveOnSurface (2D curve on surface)
    │   │   └── BRep_PolygonOnTriangulation
    │   └── Tolerance, flags (SameParameter, SameRange, Degenerated)
    ├── TopoDS_TWire (list of edges)
    ├── TopoDS_TFace → BRep_TFace
    │   ├── Handle(Geom_Surface)
    │   ├── Poly_ListOfTriangulation (optional)
    │   ├── TopLoc_Location
    │   └── Standard_Real tolerance
    └── TopoDS_TShell (list of faces)
```

#### 3.2.1 Key OCCT Classes

**TopoDS_Shape** (wrapper):
```cpp
class TopoDS_Shape {
  Handle(TopoDS_TShape) myTShape;      // Immutable topology
  TopLoc_Location myLocation;          // Local coordinate system
  TopAbs_Orientation myOrient;         // FORWARD/REVERSED
};
```

**BRep_TEdge**:
```cpp
class BRep_TEdge : public TopoDS_TEdge {
  Standard_Real myTolerance;
  Standard_Integer myFlags;  // SameParameter, SameRange, Degenerated
  BRep_ListOfCurveRepresentation myCurves;  // Multiple representations
};
```

**BRep_TFace**:
```cpp
class BRep_TFace : public TopoDS_TFace {
  Handle(Geom_Surface) mySurface;
  TopLoc_Location myLocation;
  Standard_Real myTolerance;
  Standard_Boolean myNaturalRestriction;
  Poly_ListOfTriangulation myTriangulations;
};
```

---

## 4. CRITICAL DIFFERENCES FOR TRANSLATION

### 4.1 Fundamental Design Differences

| Aspect | OpenNURBS | OpenCASCADE |
|--------|-----------|-------------|
| **Array indexing** | 0-based | 1-based (HArray) |
| **CV memory layout** | Flat stride-based array | Handle to 1D/2D array (OOP) |
| **Weight storage** | Homogeneous coordinates | Separate weight array |
| **Knot representation** | Simple array with implicit multiplicities | Explicit knots + multiplicities |
| **Topology mutability** | Mutable objects | Immutable TShape + mutable wrapper |
| **Orientation** | Explicit m_bRev flags | TopAbs_Orientation in wrapper |
| **Location** | Implicit in geometry | Explicit TopLoc_Location |
| **Curve representations** | Single curve per edge | List of representations |
| **Dimension** | Variable (m_dim) | Fixed 3D (gp_Pnt) |

### 4.2 Data Access Pattern Differences

**OpenNURBS** (imperative, pointer-based):
```cpp
// Access control point i
double* cv_i = nurbscurve.m_cv + i * nurbscurve.m_cv_stride;
double x = cv_i[0], y = cv_i[1], z = cv_i[2];
double weight = (nurbscurve.m_is_rat) ? cv_i[3] : 1.0;
```

**OpenCASCADE** (OOP, handle-based):
```cpp
// Access control point i (1-indexed)
gp_Pnt pnt = bsplinecurve->Pole(i);
double weight = (bsplinecurve->IsRational()) ? bsplinecurve->Weight(i) : 1.0;
```

### 4.3 B-Rep Topology Differences

**OpenNURBS**: Flat indexed arrays with explicit cross-references
- Direct array indexing: `m_V[vi]`, `m_E[ei]`, `m_F[fi]`
- Explicit pointers: `edge.m_c3i`, `trim.m_c2i`, `trim.m_ei`
- Single curve per edge
- 2D trim curves in m_C2[]

**OpenCASCADE**: Hierarchical, immutable inner + mutable outer wrapper
- Handles to immutable TShape
- Multiple curve representations per edge (3D, on-surface, polyline)
- Explicit location transformation
- Implicit orientation in wrapper

---

## 5. DETAILED MEMBER DATA MAPPING

### Curve: OpenNURBS → OCCT

| OpenNURBS | OCCT | Notes |
|-----------|------|-------|
| `m_is_rat` | `IsRational()` | Boolean flag |
| `m_order` | `Degree() + 1` | Relationship |
| `m_cv_count` | `NbPoles()` | CP count |
| `m_cv` (flattened) | `Poles()` (HArray) | Layout change |
| `m_knot` | `Knots()` + `Mults()` | Representation change |
| `m_cv_stride` | Implicit in HArray | Absorbed by API |
| Weights (in CV) | `Weights()` (HArray) | Separate array |

### Surface: OpenNURBS → OCCT

| OpenNURBS | OCCT | Notes |
|-----------|------|-------|
| `m_order[0]` | `UDegree() + 1` | U degree |
| `m_order[1]` | `VDegree() + 1` | V degree |
| `m_cv_count[0]` | `NbUPoles()` | U count |
| `m_cv_count[1]` | `NbVPoles()` | V count |
| `m_cv` (2D flat) | `Poles()` (HArray2) | 2D grid |
| `m_knot[0]` | `UKnots()` + `UMults()` | U representation |
| `m_knot[1]` | `VKnots()` + `VMults()` | V representation |
| `m_cv_stride[0]`, `[1]` | Implicit | Absorbed by API |

### B-Rep: Topology Mapping

| OpenNURBS | OCCT | Notes |
|-----------|------|-------|
| `ON_Brep` | `TopoDS_Shape` (top) | Different hierarchy |
| `m_V[]` | `TopoDS_Vertex` | Vertex wrapper |
| `m_E[]` | `TopoDS_Edge` | Edge wrapper |
| `m_F[]` | `TopoDS_Face` | Face wrapper |
| `m_C3[]` | In `BRep_Curve3D` | Via edge representation list |
| `m_C2[]` | In `BRep_CurveOnSurface` | Via edge representation list |
| `m_S[]` | In `BRep_TFace` | Surface via face |
| `m_bRev` flags | `TopAbs_Orientation` | In wrapper |
| Single curve | Multiple representations | List model |

---

## 6. MEMORY LAYOUT EXAMPLES

### OpenNURBS Rational Cubic Curve

```
m_dim = 3, m_is_rat = 1, m_order = 4, m_cv_count = 5
m_cv_stride = 4 (3D + weight)

m_cv array:
[x0w0] [y0w0] [z0w0] [w0] | [x1w1] [y1w1] [z1w1] [w1] | ... | [x4w4] [y4w4] [z4w4] [w4]

m_knot array (8 = 4 + 5 - 2):
[k0] [k1] [k2] [k3] [k4] [k5] [k6] [k7]

Interior domain: [k2, k5]
```

### OCCT B-Spline Curve (Same Curve)

```
poles = HArray1OfPnt [1..5] {
  gp_Pnt(1) = {x0, y0, z0}
  gp_Pnt(2) = {x1, y1, z1}
  ...
  gp_Pnt(5) = {x4, y4, z4}
}

weights = HArray1OfReal [1..5] {w0, w1, w2, w3, w4}

uknots = HArray1OfReal [1..3] {k0, k2, k5}  (unique)
umults = HArray1OfInteger [1..3] {4, 1, 4}  (clamped)

flatknots = HArray1OfReal [1..8] {k0, k0, k0, k0, k2, k5, k5, k5, k5}
```

---

## 7. KEY API METHODS REFERENCE

### OpenNURBS Curve API

```cpp
// Creation
bool Create(int dimension, bool bIsRational, int order, int cv_count);
bool CreateClampedUniformNurbs(int dimension, int order, int point_count, 
                                const ON_3dPoint* point, double knot_delta = 1.0);

// Access
int Degree() const;
int CVCount() const;
ON_Interval Domain() const;
const ON_3dPoint& PointAt(double t) const;

// Modification
bool SetDomain(double t0, double t1);
bool SetCV(int i, const ON_4dPoint& cv);
bool ChangeClosedCurveSeam(double t);

// Evaluation
bool Evaluate(double t, int der_count, int v_stride, double* v);
```

### OCCT Curve API

```cpp
// Creation
Geom_BSplineCurve(const TColgp_Array1OfPnt& Poles,
                 const TColStd_Array1OfReal& Knots,
                 const TColStd_Array1OfInteger& Mults,
                 const Standard_Integer Degree,
                 const Standard_Boolean Periodic = false);

// Access
Standard_Integer Degree() const;
Standard_Integer NbPoles() const;
const gp_Pnt& Pole(Standard_Integer Index) const;
Standard_Real Weight(Standard_Integer Index) const;
Standard_Real Knot(Standard_Integer Index) const;

// Modification
void SetPole(Standard_Integer Index, const gp_Pnt& P);
void SetWeight(Standard_Integer Index, Standard_Real Weight);
void SetKnot(Standard_Integer Index, Standard_Real K);

// Evaluation (from base class)
gp_Pnt Value(Standard_Real U);
void D0(Standard_Real U, gp_Pnt& P);
void D1(Standard_Real U, gp_Pnt& P, gp_Vec& V);
```

---

## 8. TRANSLATION CHECKLIST

### For NURBS Curves
- [ ] Convert 0-based array indexing to 1-based
- [ ] Extract weights from homogeneous coordinates
- [ ] Expand implicit knot multiplicities to explicit form
- [ ] Handle dimension mismatch (pad 3D or truncate higher dimensions)
- [ ] Verify knot vector properties (non-decreasing, correct length)
- [ ] Map domain correctly

### For NURBS Surfaces
- [ ] Convert 2D flattened array to 2D HArray structure
- [ ] Handle separate weight arrays
- [ ] Expand knot multiplicities for both U and V
- [ ] Verify 2D array bounds and stride calculations
- [ ] Handle parametric space orientation (parametric domain mapping)

### For B-Rep Topology
- [ ] Create shape wrappers (TopoDS_Vertex, Edge, Face)
- [ ] Map vertex indices and coordinates
- [ ] Create edge topology and attach 3D curves
- [ ] Create face topology and attach surfaces
- [ ] Handle trim curves (2D in parameter space)
- [ ] Manage loops and their orientation
- [ ] Preserve tolerance information
- [ ] Handle orientation flags (forward/reversed)

---

## 9. REFERENCES

- **OpenNURBS 8.24**: `opennurbs_brep.h`, `opennurbs_nurbscurve.h`, `opennurbs_nurbssurface.h`
- **OpenCASCADE 7.9.3**: `Geom/Geom_BSplineCurve.hxx`, `Geom/Geom_BSplineSurface.hxx`, `BRep/BRep_T*.hxx`

