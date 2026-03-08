cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# step2brep — standalone STEP → .brep converter
#
# Reads STEP files with OCCT's STEPControl_Reader and writes the extracted
# solid/shell shapes as OCCT .brep files for use with test_occ_roundtrip.
#
# Requires STEP-capable OCCT (DataExchange module).  The same set of OCCT
# toolkits used by test_step_roundtrip is referenced here.
# ---------------------------------------------------------------------------

# Re-use the _STEP_LINK_LIBS variable defined in the parent (tests/CMakeLists.txt
# is a sibling, so we define the same set here rather than depending on it).
set(_STEP2BREP_LINK_LIBS
    TKDESTEP TKXSBase TKDE TKDECascade
    TKXCAF
    TKCDF TKLCAF TKCAF
    TKBinL TKXmlL TKBin TKXml TKStdL TKStd
    TKTObj TKBinTObj TKXmlTObj TKVCAF
    TKBinXCAF TKXmlXCAF
    TKService TKV3d
    TKExpress
    # Modeling / geometry
    TKBRep TKTopAlgo TKGeomAlgo TKGeomBase TKG3d TKG2d TKMath TKernel
    TKShHealing TKBool TKBO
)

add_executable(step2brep step2brep.cpp)

target_include_directories(step2brep PRIVATE ${OCCT_INCLUDE_DIRS})

target_link_libraries(step2brep PRIVATE ${_STEP2BREP_LINK_LIBS})

# Install alongside other tools/tests in the binary directory (no install rule
# needed — just leave it in the build tree).
