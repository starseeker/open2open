cmake_minimum_required(VERSION 3.16)
project(open2open VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Paths relative to this file's directory
# ---------------------------------------------------------------------------
set(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(OPENNURBS_SRC  "${REPO_ROOT}/opennurbs-8.24.25281.15001")
set(OCCT_SRC       "${REPO_ROOT}/OCCT-7_9_3")

# ---------------------------------------------------------------------------
# 1. openNURBS  — add as a sub-project to get the opennurbsStatic target
# ---------------------------------------------------------------------------
if(NOT TARGET opennurbsStatic)
    add_subdirectory("${OPENNURBS_SRC}"
                     "${CMAKE_CURRENT_BINARY_DIR}/opennurbs"
                     EXCLUDE_FROM_ALL)
endif()

# ---------------------------------------------------------------------------
# 2. OpenCASCADE  — search order:
#    a) caller-supplied OpenCASCADE_DIR (highest priority)
#    b) pre-built artifacts checked into build_artifacts/occt/
#    c) system-installed package
#    d) bundled OCCT-7_9_3 source tree (slowest — full recompile)
# ---------------------------------------------------------------------------
set(_BUNDLED_OCCT_CMAKE
    "${REPO_ROOT}/build_artifacts/occt/lib/cmake/opencascade-7.9.3")

if(NOT OpenCASCADE_DIR AND NOT OpenCASCADE_FOUND
   AND EXISTS "${_BUNDLED_OCCT_CMAKE}/OpenCASCADEConfig.cmake")
    set(OpenCASCADE_DIR "${_BUNDLED_OCCT_CMAKE}" CACHE PATH
        "OpenCASCADE cmake config dir (auto-set from build_artifacts/occt)" FORCE)
    message(STATUS
        "open2open: using pre-built OCCT from build_artifacts/occt/ "
        "(set OpenCASCADE_DIR to override)")
endif()

find_package(OpenCASCADE QUIET)

if(OpenCASCADE_FOUND)
    message(STATUS "open2open: using installed OpenCASCADE ${OpenCASCADE_VERSION}")
    set(OCCT_INCLUDE_DIRS  "${OpenCASCADE_INCLUDE_DIR}")
    set(OCCT_LINK_LIBS
        ${OpenCASCADE_FoundationClasses_LIBRARIES}
        ${OpenCASCADE_ModelingAlgorithms_LIBRARIES}
        ${OpenCASCADE_ModelingData_LIBRARIES}
    )
    # Some packaged versions of OCCT are missing headers introduced in later
    # releases (e.g. NCollection_AliasedArray.hxx introduced in 7.7).
    # A compatibility shim lives in open2open/compat/ and must be found before
    # the system opencascade include directory.
    list(INSERT OCCT_INCLUDE_DIRS 0
         "${CMAKE_CURRENT_SOURCE_DIR}/compat")
else()
    message(STATUS "open2open: OpenCASCADE package not found – using bundled source tree")

    # When OCCT is being built as a sub-project the exported targets land in
    # the binary tree; just add it and let CMake wire up includes/links.
    if(NOT TARGET TKBRep)
        add_subdirectory("${OCCT_SRC}"
                         "${CMAKE_CURRENT_BINARY_DIR}/OCCT"
                         EXCLUDE_FROM_ALL)
    endif()

    # Collect the minimal set of OCCT toolkits we need
    set(OCCT_LINK_LIBS
        TKMath
        TKernel
        TKG2d
        TKG3d
        TKGeomBase
        TKBRep
        TKTopAlgo
        TKGeomAlgo
        TKBool
    )
    # Header directories are exposed via target includes on the OCCT targets;
    # no manual OCCT_INCLUDE_DIRS needed when building in-tree.
    set(OCCT_INCLUDE_DIRS "")
endif()

# ---------------------------------------------------------------------------
# 3. open2open library
# ---------------------------------------------------------------------------

# Detect XCAF availability.
#
# The build_artifacts/occt package (the default) always provides both the
# ApplicationFramework module (TKLCAF, TKCAF, TKCDF) and the DataExchange
# module (TKXCAF).  The bundled source build (BUILD_MODULE_ApplicationFramework=OFF)
# does NOT; in that case _HAVE_XCAF stays OFF and attrs_convert is skipped.
#
# XCAF libs required by attrs_convert.cpp:
#   TKLCAF  — TDF_Label, TDataStd_Name, TDocStd_Document
#   TKCDF   — CDM framework (base of TDocStd_Document)
#   TKXCAF  — XCAFDoc_* tools (ShapeTool, ColorTool, LayerTool, MaterialTool)
#
# TKService / TKV3d are NOT needed here — we intentionally avoid the
# XCAFDoc_VisMaterialTool path that would pull in Graphic3d.
set(_HAVE_XCAF OFF)
if(OpenCASCADE_FOUND AND DEFINED OpenCASCADE_ApplicationFramework_LIBRARIES)
    # Pre-built OCCT (build_artifacts/ or installed): ApplicationFramework
    # module guarantees TKLCAF/TKCAF/TKCDF; DataExchange guarantees TKXCAF.
    set(_HAVE_XCAF ON)
    set(_XCAF_LINK_LIBS TKLCAF TKCAF TKCDF TKXCAF)
    message(STATUS "open2open: XCAF support enabled (pre-built OCCT)")
elseif(TARGET TKLCAF AND TARGET TKXCAF)
    # In-tree bundled OCCT build that was configured WITH ApplicationFramework.
    set(_HAVE_XCAF ON)
    set(_XCAF_LINK_LIBS TKLCAF TKCAF TKCDF TKXCAF)
    message(STATUS "open2open: XCAF support enabled (bundled OCCT)")
else()
    message(STATUS "open2open: XCAF support disabled (ApplicationFramework/DataExchange not available)")
endif()

# Core sources (always compiled)
set(_OPEN2OPEN_SOURCES
    src/geom_convert.cpp
    src/brep_convert.cpp
    src/mesh_convert.cpp
)
# attrs_convert.cpp requires XCAF
if(_HAVE_XCAF)
    list(APPEND _OPEN2OPEN_SOURCES src/attrs_convert.cpp)
endif()

# fcstd_convert.cpp requires libzip (for ZIP reading) + OCCT LDOM (TKernel).
# LDOM is always part of TKernel so no extra link flags are needed beyond -lzip.
# The module is also optional: silently skip if libzip is not installed.
find_library(_LIBZIP_LIB NAMES zip DOC "libzip library")
find_path(_LIBZIP_INC zip.h DOC "libzip header directory")
# fcstd_convert.cpp is always compiled: ParseDiffuseColors has no zip dependency.
# The full ReadFcstdDoc function requires libzip and is guarded by
# OPEN2OPEN_HAVE_LIBZIP in the source.
list(APPEND _OPEN2OPEN_SOURCES src/fcstd_convert.cpp)
if(_LIBZIP_LIB AND _LIBZIP_INC)
    set(_HAVE_LIBZIP ON)
    message(STATUS "open2open: FreeCAD FCStd support enabled (libzip found: ${_LIBZIP_LIB})")
else()
    set(_HAVE_LIBZIP OFF)
    message(STATUS "open2open: FreeCAD FCStd support disabled (libzip not found)")
endif()

add_library(open2open STATIC ${_OPEN2OPEN_SOURCES})

target_include_directories(open2open
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${OPENNURBS_SRC}"
        ${OCCT_INCLUDE_DIRS}
)

target_link_libraries(open2open
    # Use a linker group to handle the circular dependency between
    # opennurbsStatic (which defines zcalloc) and its bundled zlib
    # (which calls zcalloc).  --start-group/--end-group causes the
    # linker to make multiple passes over the grouped archives.
    PUBLIC
        -Wl,--start-group
        opennurbsStatic
        -Wl,--end-group
    PRIVATE ${OCCT_LINK_LIBS}
)

# Expose XCAF availability to all consumers of the library.
if(_HAVE_XCAF)
    target_compile_definitions(open2open PUBLIC OPEN2OPEN_HAVE_XCAF)
    # TKCAF (TNaming.cxx) has an undeclared dependency on TKBO
    # (IntTools_FClass2d) that is absent from TKCAF's cmake
    # INTERFACE_LINK_LIBRARIES.  TKBO is also in OCCT_LINK_LIBS but
    # appears before TKCAF.  Using LINK_GROUP:RESCAN wraps these libraries
    # in --start-group/--end-group so the linker rescans them and resolves
    # the forward reference from TKCAF to TKBO.
    target_link_libraries(open2open PRIVATE
        "$<LINK_GROUP:RESCAN,${_XCAF_LINK_LIBS},TKBO>"
    )
endif()

# Expose FCStd / libzip availability to all consumers of the library.
if(_HAVE_LIBZIP)
    target_compile_definitions(open2open PUBLIC OPEN2OPEN_HAVE_LIBZIP)
    target_include_directories(open2open PRIVATE "${_LIBZIP_INC}")
    target_link_libraries(open2open PRIVATE "${_LIBZIP_LIB}")
endif()

# ---------------------------------------------------------------------------
# 4. Tests
# ---------------------------------------------------------------------------
enable_testing()
add_subdirectory(tests)

# ---------------------------------------------------------------------------
# 5. Tools (step2brep converter, etc.)
# ---------------------------------------------------------------------------
# Only build tools when STEP-capable OCCT is available (DataExchange module).
# build_artifacts/occt provides TKDESTEP; if it's absent we skip the tools
# subdirectory silently so plain builds still work.
if(TARGET TKDESTEP OR EXISTS "${REPO_ROOT}/build_artifacts/occt/lib/libTKDESTEP.a")
    add_subdirectory(tools)
endif()
