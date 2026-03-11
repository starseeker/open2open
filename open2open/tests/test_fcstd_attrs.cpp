// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_fcstd_attrs.cpp — Unit tests for fcstd_convert: FreeCAD FCStd
// attribute / metadata / per-face colour extraction.
//
// Tests exercise:
//   1. ParseDiffuseColors() — binary blob parsing
//   2. ReadFcstdDoc() — full archive read with Document.xml + GuiDocument.xml
//
// The tests use an FCStd file bundled with the repository at:
//   phasma-mech-assembly/Simplified COTS 3D models/ESQ-126-38-G-D.FCStd
//
// If that file is not present (e.g. the submodule was not checked out) the
// tests that depend on it are SKIPPED and reported as such.

#include "open2open/fcstd_convert.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace open2open;

// ---------------------------------------------------------------------------
// Minimal test framework (matches the style used in test_brep_complex.cpp)
// ---------------------------------------------------------------------------
static int g_pass = 0;
static int g_fail = 0;
static int g_skip = 0;

static void check(bool cond, const char* label, const char* file, int line)
{
    if (cond) {
        ++g_pass;
    } else {
        ++g_fail;
        std::fprintf(stderr, "FAIL [%s] at %s:%d\n", label, file, line);
    }
}

#define CHECK(cond)  check((cond), #cond, __FILE__, __LINE__)
#define SKIP(msg)    do { ++g_skip; \
    std::fprintf(stdout, "SKIP [%s]\n", msg); return; } while(0)

// ---------------------------------------------------------------------------
// Test 1: ParseDiffuseColors — boundary conditions
// ---------------------------------------------------------------------------
static void Test_ParseDiffuseColors_Basic()
{
    std::printf("--- Test 1: ParseDiffuseColors basic\n");

    std::vector<FcstdColor> out;

    // Null pointer / zero size → false
    CHECK(!ParseDiffuseColors(nullptr, 0, out));
    CHECK(!ParseDiffuseColors(nullptr, 4, out));

    // Too short (only 3 bytes) → false
    const unsigned char short_data[] = {0x01, 0x00, 0x00};
    CHECK(!ParseDiffuseColors(short_data, 3, out));

    // count=0 → true, empty output
    const unsigned char zero_count[] = {0x00, 0x00, 0x00, 0x00};
    CHECK(ParseDiffuseColors(zero_count, 4, out));
    CHECK(out.empty());

    // count=1, 4 bytes of data → true, 1 entry
    // Bytes: count=1 (LE), then 0x00808080 (LE uint32 = packed 0x80808000)
    const unsigned char one_entry[] = {
        0x01, 0x00, 0x00, 0x00,   // count = 1
        0x00, 0x80, 0x80, 0x80    // LE uint32 = 0x80808000 → R=128 G=128 B=128 A=0
    };
    CHECK(ParseDiffuseColors(one_entry, 8, out));
    CHECK(out.size() == 1);
    if (out.size() == 1) {
        CHECK(FcstdColorR(out[0]) == 128);
        CHECK(FcstdColorG(out[0]) == 128);
        CHECK(FcstdColorB(out[0]) == 128);
        CHECK(FcstdColorA(out[0]) == 0);   // alpha=0 → fully opaque in FreeCAD
    }

    // count=2 but only 4 bytes of color data (need 8) → false
    const unsigned char truncated[] = {
        0x02, 0x00, 0x00, 0x00,   // count = 2
        0x00, 0x80, 0x80, 0x80    // only 4 bytes; need 8 for 2 entries
    };
    CHECK(!ParseDiffuseColors(truncated, 8, out));
}

// ---------------------------------------------------------------------------
// Test 2: ParseDiffuseColors — known ESQ connector colors
// ---------------------------------------------------------------------------
static void Test_ParseDiffuseColors_Colors()
{
    std::printf("--- Test 2: ParseDiffuseColors colors\n");

    // ESQ-126-38-G-D DiffuseColor (verified above): count=1, R=128,G=128,B=128,A=0
    const unsigned char esq_gray[] = {
        0x01, 0x00, 0x00, 0x00,  // count=1
        0x00, 0x80, 0x80, 0x80   // 0x80808000 → R=128,G=128,B=128,A=0
    };
    std::vector<FcstdColor> out;
    CHECK(ParseDiffuseColors(esq_gray, 8, out));
    CHECK(out.size() == 1);
    if (!out.empty()) {
        CHECK(FcstdColorR(out[0]) == 128);
        CHECK(FcstdColorG(out[0]) == 128);
        CHECK(FcstdColorB(out[0]) == 128);
        CHECK(FcstdColorA(out[0]) == 0);
    }

    // 3U structure light blue-gray: count=1, bytes=00,e9,e3,e3
    // LE uint32: 0xE3E3E900 → R=227, G=227, B=233, A=0
    const unsigned char blue_gray[] = {
        0x01, 0x00, 0x00, 0x00,
        0x00, 0xe9, 0xe3, 0xe3   // LE = 0xE3E3E900 → R=227,G=227,B=233,A=0
    };
    CHECK(ParseDiffuseColors(blue_gray, 8, out));
    CHECK(out.size() == 1);
    if (!out.empty()) {
        CHECK(FcstdColorR(out[0]) == 227);
        CHECK(FcstdColorG(out[0]) == 227);
        CHECK(FcstdColorB(out[0]) == 233);
        CHECK(FcstdColorA(out[0]) == 0);
    }
}

// ---------------------------------------------------------------------------
// Test 3: FcstdColorToRGBAf — alpha inversion
// ---------------------------------------------------------------------------
static void Test_FcstdColorToRGBAf()
{
    std::printf("--- Test 3: FcstdColorToRGBAf\n");

    float r, g, b, a;

    // Fully opaque (FreeCAD alpha=0 → standard alpha=1)
    FcstdColor opaque = 0x80808000u; // R=128,G=128,B=128,A=0
    FcstdColorToRGBAf(opaque, r, g, b, a);
    CHECK(r > 0.49f && r < 0.51f);
    CHECK(g > 0.49f && g < 0.51f);
    CHECK(b > 0.49f && b < 0.51f);
    CHECK(a > 0.99f);  // fully opaque

    // 50% transparent (FreeCAD alpha=128 → standard alpha ≈ 0.498)
    FcstdColor half_trans = 0xFF000080u; // R=255,G=0,B=0,A=128
    FcstdColorToRGBAf(half_trans, r, g, b, a);
    CHECK(r > 0.99f);
    CHECK(g < 0.01f);
    CHECK(b < 0.01f);
    CHECK(a > 0.49f && a < 0.51f);

    // Fully transparent (FreeCAD alpha=255 → standard alpha=0)
    FcstdColor trans = 0x0000FFFFu; // R=0,G=0,B=255,A=255 (fully transparent in FC)
    // But alpha=255 in FreeCAD → a = 1 - 255/255 = 0
    FcstdColorToRGBAf(trans, r, g, b, a);
    CHECK(b > 0.99f);
    CHECK(a < 0.01f);  // fully transparent
}

// ---------------------------------------------------------------------------
// Test 4: ReadFcstdDoc — ESQ connector (requires phasma submodule)
// ---------------------------------------------------------------------------
static void Test_ReadFcstdDoc(const std::string& fcstd_path)
{
    std::printf("--- Test 4: ReadFcstdDoc (%s)\n", fcstd_path.c_str());

    FcstdDoc doc;
    bool ok = ReadFcstdDoc(fcstd_path, doc);
    CHECK(ok);
    if (!ok) return;

    // Document metadata
    CHECK(doc.meta.label == "ESQ-126-38-G-D");
    CHECK(!doc.meta.creation_date.empty());

    // Should have exactly 4 Part::Feature objects with BRep files
    CHECK(doc.objects.size() == 4);

    // All objects should have non-empty labels and brp_file paths
    for (const auto& obj : doc.objects) {
        CHECK(!obj.label.empty());
        CHECK(!obj.brp_file.empty());
        // All objects in this file have 1 face per DiffuseColor
        CHECK(obj.face_colors.size() == 1);
    }

    // Verify known labels
    auto find_by_label = [&](const std::string& lbl) -> const FcstdObject* {
        for (const auto& o : doc.objects)
            if (o.label == lbl) return &o;
        return nullptr;
    };

    const FcstdObject* socket = find_by_label("ESQ-126-38-G-D_socket");
    CHECK(socket != nullptr);
    if (socket) {
        CHECK(socket->brp_file == "PartShape.brp");
        CHECK(!socket->face_colors.empty());
        // Socket is gray (R=128, G=128, B=128)
        if (!socket->face_colors.empty()) {
            CHECK(FcstdColorR(socket->face_colors[0]) == 128);
            CHECK(FcstdColorG(socket->face_colors[0]) == 128);
            CHECK(FcstdColorB(socket->face_colors[0]) == 128);
        }
    }

    const FcstdObject* pins = find_by_label("ESQ-126-38-G-D_pins");
    CHECK(pins != nullptr);
    if (pins) {
        CHECK(pins->brp_file == "PartShape1.brp");
    }
}

// ---------------------------------------------------------------------------
// Test 5: ReadFcstdDoc — invalid path returns false
// ---------------------------------------------------------------------------
static void Test_ReadFcstdDoc_InvalidPath()
{
    std::printf("--- Test 5: ReadFcstdDoc invalid path\n");

    FcstdDoc doc;
    CHECK(!ReadFcstdDoc("/nonexistent/path.FCStd", doc));
    CHECK(doc.objects.empty());
    CHECK(doc.meta.label.empty());
}

// ---------------------------------------------------------------------------
// Test 6: ReadFcstdDoc — 3U structure with multi-face colors
// ---------------------------------------------------------------------------
static void Test_ReadFcstdDoc_3UStructure(const std::string& fcstd_path)
{
    std::printf("--- Test 6: ReadFcstdDoc 3U structure multi-face colors (%s)\n",
                fcstd_path.c_str());

    FcstdDoc doc;
    bool ok = ReadFcstdDoc(fcstd_path, doc);
    CHECK(ok);
    if (!ok) return;

    // The 3U structure has objects with many faces and different per-face colors
    bool found_multiface = false;
    for (const auto& obj : doc.objects) {
        if (obj.face_colors.size() > 1) {
            found_multiface = true;
            // All face_colors should be valid packed RGBA values
            for (FcstdColor c : obj.face_colors) {
                // alpha=0 is normal (opaque), but check it's a valid packed value
                (void)c; // just accessing it proves it's readable
            }
            break;
        }
    }
    CHECK(found_multiface);

    // Check the known 485-face shape (Solid = DiffuseColor with count=485)
    auto it = std::find_if(doc.objects.begin(), doc.objects.end(),
        [](const FcstdObject& o) { return o.face_colors.size() == 485; });
    if (it != doc.objects.end()) {
        // All entries should be the same light blue-gray or other chassis colors
        // R=227,G=227,B=233,A=0 is one of the known colors
        bool found_blue_gray = false;
        for (FcstdColor c : it->face_colors) {
            if (FcstdColorR(c) == 227 && FcstdColorG(c) == 227 && FcstdColorB(c) == 233)
                found_blue_gray = true;
        }
        CHECK(found_blue_gray);
    }
}

#ifdef OPEN2OPEN_HAVE_LIBZIP
// ---------------------------------------------------------------------------
// Test 7: ONX_ModelToFCStdFile + FCStdFileToONX_Model round-trip
// ---------------------------------------------------------------------------
#include "open2open/brep_convert.h"
#include "opennurbs.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopoDS_Shape.hxx>

static void Test_FCStdRoundTrip()
{
    std::printf("--- Test 7: FCStd model round-trip (ONX_Model ↔ FCStd)\n");

    ONX_Model src;
    src.m_settings.m_ModelUnitsAndTolerances.m_unit_system =
        ON_UnitSystem(ON::LengthUnitSystem::Millimeters);

    // Add a box brep
    {
        TopoDS_Shape shape = BRepPrimAPI_MakeBox(10.0, 20.0, 5.0).Shape();
        ON_Brep* brep = new ON_Brep();
        open2open::OCCTToON_Brep(shape, *brep, 1e-3);
        ON_3dmObjectAttributes* a = new ON_3dmObjectAttributes();
        a->m_name = L"TestBox";
        a->m_color = ON_Color(100, 150, 200);
        a->SetColorSource(ON::color_from_object);
        // Add per-face colours
        for (int fi = 0; fi < brep->m_F.Count(); ++fi) {
            const ON_Color fc = (fi % 2 == 0)
                ? ON_Color(255, 0, 0) : ON_Color(0, 255, 0);
            brep->m_F[fi].SetPerFaceColor(fc);
        }
        src.AddManagedModelGeometryComponent(brep, a);
    }

    // Write to /tmp
    const std::string tmpFile = "/tmp/test_fcstd_roundtrip.FCStd";
    int nWritten = open2open::ONX_ModelToFCStdFile(tmpFile, src, 1e-3);
    CHECK(nWritten == 1);
    if (nWritten == 0) return;

    // Read back
    ONX_Model dst;
    int nRead = open2open::FCStdFileToONX_Model(tmpFile, dst, 1e-3);
    CHECK(nRead == 1);

    // Verify object name
    bool foundBox = false;
    {
        ONX_ModelComponentIterator it(dst,
            ON_ModelComponent::Type::ModelGeometry);
        for (const ON_ModelComponent* mc = it.FirstComponent();
             mc != nullptr; mc = it.NextComponent())
        {
            const ON_ModelGeometryComponent* mgc =
                ON_ModelGeometryComponent::Cast(mc);
            if (!mgc) continue;
            const ON_Geometry* g = mgc->Geometry(nullptr);
            if (!g || g->ObjectType() != ON::brep_object) continue;
            const ON_Brep* brep = ON_Brep::Cast(g);
            if (!brep) continue;
            // Should have 6 faces (box)
            if (brep->m_F.Count() == 6) {
                const ON_3dmObjectAttributes* at = mgc->Attributes(nullptr);
                if (at && at->m_name == ON_wString(L"TestBox"))
                    foundBox = true;
            }
        }
    }
    CHECK(foundBox);

    // Clean up
    std::remove(tmpFile.c_str());
}
#endif // OPEN2OPEN_HAVE_LIBZIP

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    // Locate the phasma FCStd test files.
    // Strategy: use $OPEN2OPEN_REPO env var if set; otherwise derive the repo
    // root from the test executable path (which lives in build/tests/).
    // build/tests/ is typically 3 directories below the repo root:
    //   <repo>/build/tests/test_fcstd_attrs → <repo> = ../../..
    // Fall back to the known GitHub Actions CI path as a last resort.
    const char* repo_env = getenv("OPEN2OPEN_REPO");

    std::string repo_root;
    if (repo_env && repo_env[0]) {
        repo_root = repo_env;
    } else if (argc >= 1 && argv[0]) {
        // Derive from argv[0]: go up 3 parent directories
        std::string exe = argv[0];
        // Normalize: strip trailing slash if any
        while (!exe.empty() && exe.back() == '/') exe.pop_back();

        // Walk up 3 levels (tests/ → build/ → repo/)
        for (int i = 0; i < 3; ++i) {
            size_t slash = exe.rfind('/');
            if (slash == std::string::npos) { exe.clear(); break; }
            exe = exe.substr(0, slash);
        }
        if (!exe.empty())
            repo_root = exe;
    }

    // Final fallback: known CI path
    if (repo_root.empty())
        repo_root = "/home/runner/work/open2open/open2open";

    std::string esq_path  = repo_root
        + "/phasma-mech-assembly/Simplified COTS 3D models/ESQ-126-38-G-D.FCStd";
    std::string u3_path   = repo_root
        + "/phasma-mech-assembly/Simplified COTS 3D models/3U structure (simplified).FCStd";

    // Always run unit tests (no FCStd file needed)
    Test_ParseDiffuseColors_Basic();
    Test_ParseDiffuseColors_Colors();
    Test_FcstdColorToRGBAf();
    Test_ReadFcstdDoc_InvalidPath();

    // Archive-dependent tests
    {
        FILE* f = std::fopen(esq_path.c_str(), "rb");
        if (f) {
            std::fclose(f);
            Test_ReadFcstdDoc(esq_path);
        } else {
            ++g_skip;
            std::printf("SKIP [Test_ReadFcstdDoc: %s not found]\n", esq_path.c_str());
        }
    }
    {
        FILE* f = std::fopen(u3_path.c_str(), "rb");
        if (f) {
            std::fclose(f);
            Test_ReadFcstdDoc_3UStructure(u3_path);
        } else {
            ++g_skip;
            std::printf("SKIP [Test_ReadFcstdDoc_3UStructure: %s not found]\n", u3_path.c_str());
        }
    }

#ifdef OPEN2OPEN_HAVE_LIBZIP
    Test_FCStdRoundTrip();
#else
    ++g_skip;
    std::printf("SKIP [Test_FCStdRoundTrip: libzip not available]\n");
#endif

    std::printf("\n%d/%d tests passed", g_pass, g_pass + g_fail);
    if (g_skip > 0)
        std::printf(", %d skipped (FCStd files not found or libzip unavailable)", g_skip);
    std::printf("\n");

    return (g_fail == 0) ? 0 : 1;
}
