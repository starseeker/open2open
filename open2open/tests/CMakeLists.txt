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
