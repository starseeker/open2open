# GitHub Copilot Instructions for open2open

## Project Overview

`open2open` is a C++17 library that converts B-Rep geometry between
[openNURBS](https://github.com/mcneel/opennurbs) (Rhino's `.3dm` format) and
[OpenCASCADE Technology (OCCT)](https://dev.opencascade.org/).  The two key
entry points are:

```cpp
TopoDS_Shape open2open::ON_BrepToOCCT(const ON_Brep&, double tol);
bool         open2open::OCCTToON_Brep(const TopoDS_Shape&, ON_Brep&, double tol);
```

## Directory Layout

```
open2open/                     ← repo root
├── open2open/                 ← library source
│   ├── src/
│   │   ├── brep_convert.cpp   ← main B-Rep conversion logic
│   │   └── geom_convert.cpp   ← NURBS curve/surface helpers
│   ├── include/open2open/
│   │   ├── brep_convert.h
│   │   └── geom_convert.h
│   └── tests/                 ← all test executables
├── opennurbs-8.24.25281.15001/ ← openNURBS source (submodule-like copy)
├── OCCT-7_9_3/                ← OpenCASCADE source (submodule-like copy)
├── test3dm/                   ← user-supplied .3dm files (flat directory)
│   └── YachtMod(55).3dm.gz    ← large model stored gzip-compressed
└── build_artifacts/           ← pre-built binaries committed to repo
    ├── libopen2open.a
    └── tests/
```

## Building

OCCT **cannot** be built as a sub-project directly (its `CMakeLists.txt`
uses `CMAKE_SOURCE_DIR` to locate `adm/cmake/*.cmake` files, which breaks
when included as a sub-project).  Always build OCCT separately first, then
point the main project at the result.

### Step 1 — Build OCCT 7.9.3

```bash
mkdir -p /tmp/occt_build
cd /tmp/occt_build
cmake /path/to/repo/OCCT-7_9_3 \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MODULE_Draw=OFF \
  -DBUILD_MODULE_Visualization=OFF \
  -DBUILD_MODULE_ApplicationFramework=OFF \
  -DBUILD_MODULE_DataExchange=OFF \
  -DBUILD_MODULE_Inspection=OFF \
  -DBUILD_LIBRARY_TYPE=Static \
  -DUSE_FREETYPE=OFF \
  -DUSE_FREEIMAGE=OFF
make -j$(nproc)
```

This produces static libraries in `/tmp/occt_build/lin64/gcc/lib/` and
headers in `/tmp/occt_build/include/opencascade/`.

### Step 2 — Create an OCCT cmake root

The generated `OpenCASCADEConfig.cmake` expects per-component target files
(e.g. `OpenCASCADEModelingAlgorithmsTargets.cmake`) that OCCT only generates
when installed — not when built in-tree.  Create them as symlinks:

```bash
mkdir -p /tmp/occt_root/lib/cmake/opencascade-7.9.3
mkdir -p /tmp/occt_root/include/opencascade
mkdir -p /tmp/occt_root/lib

cp -r /tmp/occt_build/include/opencascade/. /tmp/occt_root/include/opencascade/
cp /tmp/occt_build/lin64/gcc/lib/libTK*.a   /tmp/occt_root/lib/
cp /tmp/occt_build/OpenCASCADE*.cmake        /tmp/occt_root/lib/cmake/opencascade-7.9.3/

for comp in FoundationClasses ModelingData ModelingAlgorithms DETools; do
  ln -sf /tmp/occt_root/lib/cmake/opencascade-7.9.3/OpenCASCADETargets.cmake \
         /tmp/occt_root/lib/cmake/opencascade-7.9.3/OpenCASCADE${comp}Targets.cmake
done
```

### Step 3 — Configure and build open2open

```bash
cd /path/to/repo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DOpenCASCADE_DIR=/tmp/occt_root/lib/cmake/opencascade-7.9.3
make -j$(nproc)
```

### One-liner for rebuilding after a /tmp wipe

If `/tmp` was cleared (e.g. new CI runner), repeat Steps 1–3.  A helper
that combines all three:

```bash
REPO=/home/runner/work/open2open/open2open
mkdir -p /tmp/occt_build && cd /tmp/occt_build
cmake $REPO/OCCT-7_9_3 -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MODULE_Draw=OFF -DBUILD_MODULE_Visualization=OFF \
  -DBUILD_MODULE_ApplicationFramework=OFF -DBUILD_MODULE_DataExchange=OFF \
  -DBUILD_MODULE_Inspection=OFF -DBUILD_LIBRARY_TYPE=Static \
  -DUSE_FREETYPE=OFF -DUSE_FREEIMAGE=OFF && make -j$(nproc)

mkdir -p /tmp/occt_root/{lib/cmake/opencascade-7.9.3,include/opencascade,lib}
cp -r /tmp/occt_build/include/opencascade/. /tmp/occt_root/include/opencascade/
cp /tmp/occt_build/lin64/gcc/lib/libTK*.a   /tmp/occt_root/lib/
cp /tmp/occt_build/OpenCASCADE*.cmake        /tmp/occt_root/lib/cmake/opencascade-7.9.3/
for comp in FoundationClasses ModelingData ModelingAlgorithms DETools; do
  ln -sf /tmp/occt_root/lib/cmake/opencascade-7.9.3/OpenCASCADETargets.cmake \
         /tmp/occt_root/lib/cmake/opencascade-7.9.3/OpenCASCADE${comp}Targets.cmake
done

cd $REPO && rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DOpenCASCADE_DIR=/tmp/occt_root/lib/cmake/opencascade-7.9.3
make -j$(nproc)
```

## Running Tests

| Test | Command | Expected |
|---|---|---|
| Unit tests (brep, curves, surfaces) | `build/open2open/tests/test_brep_complex` | 14/14 |
| OCCT .brep round-trip | `build/open2open/tests/test_occ_roundtrip OCCT-7_9_3/data/occ` | 35/37 |
| openNURBS 3dm examples | `build/open2open/tests/test_3dm_examples opennurbs-8.24.25281.15001/example_files` | 1204/1209 |
| test3dm round-trip | `build/open2open/tests/test_3dm_roundtrip test3dm` | 9425/9431 |

The `.3dm.gz` file (`YachtMod(55).3dm.gz`) is decompressed automatically at
runtime by `test_3dm_roundtrip` using `gzip -d -c`.

## Key Conventions in `brep_convert.cpp`

- **SetProxyCurve vs SetProxyCurveDomain**: `ON_BrepTrim` inherits from
  `ON_CurveProxy`.  Always use the two-arg `SetProxyCurve(c2, interval)` to
  set both `m_this_domain` and `m_real_curve_domain` atomically.  Calling
  `SetProxyCurveDomain` alone only updates `m_real_curve_domain`, causing
  `ON_Brep::IsValid()`'s loop-gap check to evaluate at the wrong parameter.

- **Seam edges**: Seam pcurves are stored in the edge-forward direction.
  Reversed-edge occurrences get their pcurve reversed before storage.
  In `ON_BrepToOCCT`, seam pcurves for reversed faces must swap C1/C2 in
  `UpdateEdge`.

- **Rational NURBS**: Always use `ON_4dPoint(x*w, y*w, z_or_0, w)` when
  setting control vertices; `SetCV(int, ON_3dPoint)` silently forces weight=1.

- **Shell vs Solid**: Wrap in `TopoDS_Solid` only when all edges have ≥2
  trims (closed shell).  Return `TopoDS_Shell` for open shells.

- **YachtMod decompression**: The test file `test3dm/YachtMod(55).3dm.gz`
  must remain gzip-compressed in the repository (too large uncompressed for
  git).  `test_3dm_roundtrip` decompresses it to `/tmp/` at runtime.
