cmake_minimum_required(VERSION 3.16)

# Tests for open2open
# Each test is a standalone executable linked against open2open.

foreach(_test test_curves test_surfaces test_brep)
    add_executable(${_test} ${_test}.cpp)
    target_link_libraries(${_test} PRIVATE open2open)
    add_test(NAME ${_test} COMMAND ${_test})
endforeach()
