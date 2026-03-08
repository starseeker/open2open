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
add_library(open2open STATIC
    src/geom_convert.cpp
    src/brep_convert.cpp
)

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

# ---------------------------------------------------------------------------
# 4. Tests
# ---------------------------------------------------------------------------
enable_testing()
add_subdirectory(tests)
