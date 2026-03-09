// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_occ_roundtrip.cpp — Round-trip test for OpenCASCADE .brep files.
//
// For each .brep file in the given directory:
//   1. Read the file with BRepTools::Read into a TopoDS_Shape.
//   2. Convert TopoDS_Shape  →  ON_Brep  (OCCTToON_Brep).
//   3. Convert ON_Brep  →  TopoDS_Shape  (ON_BrepToOCCT).
//   4. Validate the round-tripped shape with BRepCheck_Analyzer.
//   5. Compare surface area (and volume for closed solids).
//
// Usage:
//   test_occ_roundtrip <path-to-brep-dir>
//
// Exit code: 0 if all .brep files round-trip successfully, 1 otherwise.

#include "open2open/brep_convert.h"

#include "opennurbs.h"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Round-trip one OCCT shape: OCCT → ON_Brep → OCCT.
// Returns true on success, fills reason on failure.
// ---------------------------------------------------------------------------
static bool RoundTripShape(const TopoDS_Shape& shape, std::string& reason)
{
    if (shape.IsNull()) { reason = "null input shape"; return false; }

    // Check that the input shape actually has faces; pure shells of wires or
    // solids without faces are not B-Rep topology we can translate.
    {
        TopExp_Explorer fex(shape, TopAbs_FACE);
        if (!fex.More()) {
            reason = "input shape has no faces";
            return false;
        }
    }

    const double kTol = 1e-6;

    // Step 1: OCCT → ON_Brep
    ON_Brep on_brep;
    if (!open2open::OCCTToON_Brep(shape, on_brep, kTol)) {
        // Call IsValid with a log to get the specific failure reason
        ON_wString wlog;
        ON_TextLog log(wlog);
        on_brep.IsValid(&log);
        ON_String slog(wlog);
        if (slog.Length() > 0 && slog.Length() < 2048) {
            reason = std::string("OCCTToON_Brep returned false: ") + slog.Array();
        } else if (slog.Length() >= 2048) {
            // Truncate at first newline to get the primary error line
            const char* p = slog.Array();
            const char* nl = strchr(p, '\n');
            if (nl && (nl - p) < 512) {
                reason = std::string("OCCTToON_Brep returned false: ") +
                         std::string(p, nl - p);
            } else {
                reason = "OCCTToON_Brep returned false";
            }
        } else {
            reason = "OCCTToON_Brep returned false";
        }
        return false;
    }
    if (!on_brep.IsValid()) {
        reason = "intermediate ON_Brep is not valid";
        return false;
    }
    if (on_brep.m_F.Count() == 0) {
        reason = "intermediate ON_Brep has no faces";
        return false;
    }

    // Step 2: ON_Brep → OCCT
    TopoDS_Shape rt_shape = open2open::ON_BrepToOCCT(on_brep, kTol);
    if (rt_shape.IsNull()) {
        reason = "ON_BrepToOCCT returned null shape";
        return false;
    }

    // Step 3: validate round-trip shape
    BRepCheck_Analyzer checker(rt_shape, Standard_False);
    if (!checker.IsValid()) {
        reason = "round-trip shape failed BRepCheck_Analyzer";
        return false;
    }

    // Verify the round-trip shape still has faces
    {
        TopExp_Explorer fex(rt_shape, TopAbs_FACE);
        if (!fex.More()) {
            reason = "round-trip shape has no faces";
            return false;
        }
    }

    // Step 4: compare surface area and volume
    GProp_GProps aProps;
    BRepGProp::SurfaceProperties(shape, aProps);
    double orig_area = aProps.Mass();
    GProp_GProps aProps2;
    BRepGProp::SurfaceProperties(rt_shape, aProps2);
    double rt_area = aProps2.Mass();
    if (orig_area > 1e-12) {
        double pct = std::fabs(rt_area - orig_area) / orig_area * 100.0;
        if (pct > 1.0) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                "surface area changed by %.4g%% (orig=%.6g, rt=%.6g)",
                pct, orig_area, rt_area);
            reason = buf;
            return false;
        }
    }
    {
        TopExp_Explorer sex(shape, TopAbs_SOLID);
        if (sex.More()) {
            GProp_GProps vProps;
            BRepGProp::VolumeProperties(shape, vProps);
            double orig_vol = vProps.Mass();
            GProp_GProps vProps2;
            BRepGProp::VolumeProperties(rt_shape, vProps2);
            double rt_vol = vProps2.Mass();
            if (std::fabs(orig_vol) > 1e-12) {
                double pct =
                    std::fabs(rt_vol - orig_vol) / std::fabs(orig_vol) * 100.0;
                if (pct > 1.0) {
                    char buf[256];
                    std::snprintf(buf, sizeof(buf),
                        "volume changed by %.4g%% (orig=%.6g, rt=%.6g)",
                        pct, orig_vol, rt_vol);
                    reason = buf;
                    return false;
                }
            }
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Test one .brep file.
// ---------------------------------------------------------------------------
struct FileResult {
    std::string path;
    bool passed = false;
    std::string reason;
};

static FileResult TestFile(const std::string& path)
{
    FileResult res;
    res.path = path;

    TopoDS_Shape shape;
    BRep_Builder builder;
    if (!BRepTools::Read(shape, path.c_str(), builder)) {
        res.reason = "BRepTools::Read failed";
        return res;
    }

    res.passed = RoundTripShape(shape, res.reason);
    return res;
}

// ---------------------------------------------------------------------------
// Helper: does a string end with the given suffix (case-insensitive)?
// ---------------------------------------------------------------------------
static bool EndsWithCI(const std::string& s, const char* suffix)
{
    std::string sfx(suffix);
    if (s.size() < sfx.size()) return false;
    std::string tail = s.substr(s.size() - sfx.size());
    for (auto& c : tail) c = (char)std::tolower((unsigned char)c);
    for (auto& c : sfx)  c = (char)std::tolower((unsigned char)c);
    return tail == sfx;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::printf("Usage: %s <path-to-brep-dir>\n", argv[0]);
        return 1;
    }

    std::string base = argv[1];

    // Collect .brep files from the directory
    std::vector<std::string> files;
    ON_FileIterator fi;
    if (fi.Initialize(base.c_str(), nullptr)) {
        for (bool ok = fi.FirstItem(); ok; ok = fi.NextItem()) {
            if (fi.CurrentItemIsDirectory()) continue;
            ON_String full(fi.CurrentItemFullPathName());
            std::string path(full.Array());
            if (EndsWithCI(path, ".brep")) {
                files.push_back(path);
            }
        }
    }

    if (files.empty()) {
        std::printf("No .brep files found in: %s\n", base.c_str());
        return 1;
    }

    std::printf("Found %d .brep files\n\n", (int)files.size());

    int total  = 0;
    int passed = 0;
    int failed = 0;

    for (const auto& f : files) {
        ++total;
        FileResult r = TestFile(f);

        std::string label = f;
        auto pos = label.rfind('/');
        if (pos != std::string::npos) label = label.substr(pos + 1);

        if (r.passed) {
            ++passed;
            std::printf("PASS [%s]\n", label.c_str());
        } else {
            ++failed;
            std::printf("FAIL [%s]: %s\n", label.c_str(), r.reason.c_str());
        }
    }

    std::printf("\n=== Summary ===\n");
    std::printf("Files: %d total, %d passed, %d failed\n", total, passed, failed);

    return (failed == 0) ? 0 : 1;
}
