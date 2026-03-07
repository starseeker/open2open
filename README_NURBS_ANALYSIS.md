# NURBS B-Rep Translation Analysis - Documentation Guide

## Overview

This directory contains comprehensive analysis documents for understanding and implementing translation between OpenNURBS and OpenCASCADE B-Rep data structures. These documents provide the foundation for the `open2open` translation tool.

## Documents in This Analysis

### 1. **NURBS_BR_REP_ANALYSIS.md** (19 KB, 539 lines)
**Purpose**: Complete technical reference documentation

**Contents**:
- **Section 1**: NURBS Curve Representation
  - OpenNURBS ON_NurbsCurve class structure and member variables
  - OpenCASCADE Geom_BSplineCurve class structure
  - Detailed storage layout comparison
  - Key API methods for each system

- **Section 2**: NURBS Surface Representation
  - OpenNURBS ON_NurbsSurface structure
  - OpenCASCADE Geom_BSplineSurface structure
  - 2D control point grid handling
  - U/V knot vector management

- **Section 3**: B-Rep Topology Structure
  - OpenNURBS flat array topology model
  - OpenCASCADE hierarchical shape model
  - Detailed class definitions for:
    - Vertices (ON_BrepVertex vs BRep_TVertex)
    - Edges (ON_BrepEdge vs BRep_TEdge)
    - Trims (ON_BrepTrim)
    - Loops (ON_BrepLoop)
    - Faces (ON_BrepFace vs BRep_TFace)

- **Section 4**: Critical Differences
  - Fundamental design differences table
  - Data access pattern comparison
  - B-Rep topology differences

- **Section 5**: Detailed Member Data Mapping
  - Curve component mapping
  - Surface component mapping
  - B-Rep topology mapping tables

- **Section 6**: Memory Layout Examples
  - Visual representation of control point storage
  - Knot vector organization
  - Rational vs non-rational storage

- **Section 7**: Key API Methods Reference
  - OpenNURBS curve/surface API
  - OCCT curve/surface API

- **Section 8**: Translation Checklist
  - Curve translation steps
  - Surface translation steps
  - B-Rep topology steps

## 2. **TRANSLATION_PLAN_SUMMARY.md** (11 KB)
**Purpose**: Implementation-focused summary with code templates

**Contents**:
- **Quick Reference**: Key differences summary tables
- **Detailed Translation Steps**:
  - Phase 1: NURBS Curves (HIGH priority)
  - Phase 2: NURBS Surfaces (HIGH priority)
  - Phase 3: B-Rep Topology (MEDIUM priority)
  - Phase 4: Data Validation
  - With complete step-by-step procedures

- **Code Templates**:
  - Curve conversion function skeleton
  - Surface conversion function skeleton
  - API quick reference for creating OCCT objects
  - Shape building with BRep_Builder

- **Index Conversion Reference**:
  - 0-based to 1-based conversion patterns
  - 2D array flattening/unflattening

- **Testing Checklist**: Validation steps

- **File References**: Quick lookup table

- **Next Steps**: Implementation roadmap

## How to Use These Documents

### For Understanding the Data Structures
1. Start with **TRANSLATION_PLAN_SUMMARY.md** "Quick Reference" section
2. Read the relevant section in **NURBS_BR_REP_ANALYSIS.md** for details
3. Refer to memory layout examples for visual understanding

### For Implementation
1. Review **TRANSLATION_PLAN_SUMMARY.md** phases in order
2. Use code templates as starting points
3. Reference **NURBS_BR_REP_ANALYSIS.md** for API method details
4. Check detailed member mapping tables for field correspondences

### For Specific Questions
- **"How do I access control points?"** → Section 1.1 & 1.2 in analysis
- **"What's the knot vector format?"** → Section 6 memory layouts
- **"How do I convert 0-based to 1-based?"** → TRANSLATION_PLAN_SUMMARY.md index conversion
- **"What B-Rep classes do I need?"** → Section 3 in analysis
- **"What are the API differences?"** → Sections 1.2, 2.2, and 7 in analysis

## Key Tables and References

### Data Type Conversions
| OpenNURBS | OCCT | Conversion Note |
|-----------|------|-----------------|
| `int m_dim` | Fixed 3D | Pad or truncate |
| `double* m_cv` | `Handle(TColgp_HArray1OfPnt)` | Array restructuring |
| `m_cv[i*stride]` | `poles->Value(i+1)` | Pointer arithmetic → API |
| Homogeneous weight | Separate `weights` array | Extract last coordinate |

### Memory Layout Differences
- **OpenNURBS**: Stride-based linear arrays (cache-efficient)
- **OCCT**: Handle-based object arrays (OOP model, 1-indexed)

### Topology Differences
- **OpenNURBS**: Flat indexed arrays (fast lookup by index)
- **OCCT**: Hierarchical shapes with immutable cores (flexible, extensible)

## Critical Implementation Notes

### Array Indexing
- OpenNURBS uses **0-based** indexing (C standard)
- OCCT uses **1-based** indexing for HArray (Fortran convention)
- Always convert: `occt_index = on_index + 1`

### Weight Handling
- **OpenNURBS**: Weights stored as last value in homogeneous coordinates
  - For rational: `CV[i] = [x*w, y*w, z*w, w]` (m_dim+1 values)
  - Extract: `weight = cv[m_dim]`
- **OCCT**: Separate weight array
  - Access: `weights->Value(i)`

### Knot Vectors
- **OpenNURBS**: Single array with implicit multiplicities (length = order+cv_count-2)
- **OCCT**: Two arrays - unique knots + multiplicities
- **Conversion**: Count consecutive values to determine multiplicity

### Dimension Handling
- **OpenNURBS**: Variable dimension `m_dim` (1, 2, 3, or higher)
- **OCCT**: Fixed 3D (`gp_Pnt`)
- **Solution**: Pad 2D with z=0, truncate higher dimensions, or extend separately

## Recommended Reading Order

**For First-Time Users**:
1. README_NURBS_ANALYSIS.md (this file)
2. TRANSLATION_PLAN_SUMMARY.md - Quick Reference section
3. NURBS_BR_REP_ANALYSIS.md - Section 1 (curves)
4. NURBS_BR_REP_ANALYSIS.md - Section 4 (differences)

**For Implementation**:
1. TRANSLATION_PLAN_SUMMARY.md - Phase 1 for curves
2. NURBS_BR_REP_ANALYSIS.md - Sections 6-7 for reference
3. TRANSLATION_PLAN_SUMMARY.md - Code templates
4. Repeat for Phase 2 (surfaces)

**For B-Rep Topology**:
1. NURBS_BR_REP_ANALYSIS.md - Section 3
2. TRANSLATION_PLAN_SUMMARY.md - Phase 3
3. Reference sections 5 for mapping tables

## Source File Locations

**OpenNURBS 8.24.25281.15001**:
```
opennurbs-8.24.25281.15001/
├── opennurbs_nurbscurve.h     (1,411 lines)
├── opennurbs_nurbssurface.h   (2,214 lines)
└── opennurbs_brep.h            (4,904 lines)
```

**OpenCASCADE 7.9.3**:
```
OCCT-7_9_3/src/
├── Geom/
│   ├── Geom_BSplineCurve.hxx
│   └── Geom_BSplineSurface.hxx
├── BRep/
│   ├── BRep_TEdge.hxx
│   ├── BRep_TFace.hxx
│   └── BRep_ListOfCurveRepresentation.hxx
└── TopoDS/
    └── TopoDS_Shape.hxx
```

## Contact & Updates

For questions or clarifications about this analysis:
1. Check the relevant section in NURBS_BR_REP_ANALYSIS.md
2. Review code templates in TRANSLATION_PLAN_SUMMARY.md
3. Refer to the original header files for implementation details
4. Test with concrete examples from both libraries

## Version Information

- **OpenNURBS**: 8.24.25281.15001
- **OpenCASCADE**: 7.9.3
- **Analysis Date**: March 2024
- **Document Status**: Complete reference analysis

---

**Note**: These documents contain extracted information from header files and are meant to supplement (not replace) reading the source code directly.
