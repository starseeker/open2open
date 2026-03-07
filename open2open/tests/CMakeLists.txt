cmake_minimum_required(VERSION 3.16)

# Tests for open2open
# Each test is a standalone executable linked against open2open.

foreach(_test test_curves test_surfaces test_brep)
    add_executable(${_test} ${_test}.cpp)
    target_link_libraries(${_test} PRIVATE open2open)
    add_test(NAME ${_test} COMMAND ${_test})
endforeach()

# Stress tests: curves and surfaces (no extra OCCT libraries needed)
foreach(_test test_curves_complex test_surfaces_complex)
    add_executable(${_test} ${_test}.cpp)
    target_link_libraries(${_test} PRIVATE open2open)
    add_test(NAME ${_test} COMMAND ${_test})
endforeach()

# Stress test: B-Rep complex requires BRepPrimAPI from TKPrim for
# OCCT-native primitive construction.
add_executable(test_brep_complex test_brep_complex.cpp)
target_link_libraries(test_brep_complex PRIVATE open2open TKPrim)
add_test(NAME test_brep_complex COMMAND test_brep_complex)

# Round-trip tests for the openNURBS 3dm example files.
# The example_files directory is expected to be at
# ${CMAKE_SOURCE_DIR}/../opennurbs-8.24.25281.15001/example_files
set(_3dm_example_dir
    "${CMAKE_SOURCE_DIR}/../opennurbs-8.24.25281.15001/example_files")
add_executable(test_3dm_examples test_3dm_examples.cpp)
target_link_libraries(test_3dm_examples PRIVATE open2open)
add_test(NAME test_3dm_examples
         COMMAND test_3dm_examples "${_3dm_example_dir}")

# Round-trip tests for user-supplied .3dm files in a flat directory.
# .3dm.gz files are decompressed on-the-fly to a temporary location.
# The test3dm directory is expected at ${CMAKE_SOURCE_DIR}/../test3dm
set(_3dm_roundtrip_dir "${CMAKE_SOURCE_DIR}/../test3dm")
add_executable(test_3dm_roundtrip test_3dm_roundtrip.cpp)
target_link_libraries(test_3dm_roundtrip PRIVATE open2open)
add_test(NAME test_3dm_roundtrip
         COMMAND test_3dm_roundtrip "${_3dm_roundtrip_dir}")

# Round-trip tests for OpenCASCADE .brep files:
#   OCCT → ON_Brep → OCCT, then validate with BRepCheck_Analyzer.
# The brep directory is expected at ${CMAKE_SOURCE_DIR}/../OCCT-7_9_3/data/occ
set(_occ_brep_dir "${CMAKE_SOURCE_DIR}/../OCCT-7_9_3/data/occ")
add_executable(test_occ_roundtrip test_occ_roundtrip.cpp)
target_link_libraries(test_occ_roundtrip PRIVATE open2open)
add_test(NAME test_occ_roundtrip
         COMMAND test_occ_roundtrip "${_occ_brep_dir}")
