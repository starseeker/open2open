// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_3dm_roundtrip.cpp — Round-trip test for .3dm files in a flat directory.
//
// Reads every .3dm file found directly in the given directory (no version
// sub-directories), then for every ON_Brep object runs the full round-trip:
//   ON_Brep  →  TopoDS_Shape  →  ON_Brep
//
// .3dm.gz files are decompressed to a temporary file and then tested; the
// temporary file is removed afterwards.
//
// Usage:
//   test_3dm_roundtrip <path-to-dir>
//
// Exit code: 0 if all tested B-Reps round-trip successfully, 1 otherwise.

#include "open2open/brep_convert.h"

#include "opennurbs.h"

#include <BRepCheck_Analyzer.hxx>
#include <TopoDS_Shape.hxx>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Attempt to round-trip one ON_Brep object.
// Returns true on success, fills reason on failure.
// ---------------------------------------------------------------------------
static bool RoundTripBrep(const ON_Brep* brep, std::string& reason)
{
    if (!brep) { reason = "null brep"; return false; }

    // Repair any obviously invalid tolerances that some old files may have
    // (e.g. ON_UNSET_VALUE tolerances).
    ON_Brep work = *brep;
    for (int i = 0; i < work.m_E.Count(); ++i) {
        double& t = work.m_E[i].m_tolerance;
        if (!ON_IsValid(t) || t < 0.0) t = 1e-6;
    }
    for (int i = 0; i < work.m_T.Count(); ++i) {
        for (int j = 0; j < 2; ++j) {
            double& t = work.m_T[i].m_tolerance[j];
            if (!ON_IsValid(t) || t < 0.0) t = 1e-6;
        }
    }
    for (int i = 0; i < work.m_V.Count(); ++i) {
        double& t = work.m_V[i].m_tolerance;
        if (!ON_IsValid(t) || t < 0.0) t = 1e-6;
    }

    if (!work.IsValid()) {
        work.Compact();
        if (!work.IsValid()) {
            reason = "input brep invalid (after tolerance repair)";
            return false;
        }
    }

    // ON → OCCT
    const double kTol = 1e-6;
    TopoDS_Shape shape = open2open::ON_BrepToOCCT(work, kTol);
    if (shape.IsNull()) {
        reason = "ON_BrepToOCCT returned null shape";
        return false;
    }

    // OCCT → ON
    ON_Brep rt;
    if (!open2open::OCCTToON_Brep(shape, rt, kTol)) {
        reason = "OCCTToON_Brep returned false";
        return false;
    }
    if (!rt.IsValid()) {
        reason = "round-trip ON_Brep is not valid";
        return false;
    }
    if (rt.m_F.Count() == 0) {
        reason = "round-trip brep has no faces";
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Test one .3dm file; returns {breps_tested, breps_passed}
// ---------------------------------------------------------------------------
struct FileResult {
    std::string path;
    int breps_tested = 0;
    int breps_passed = 0;
    int non_brep_objects = 0;
    std::vector<std::string> failures;
};

static FileResult TestFile(const std::string& path)
{
    FileResult res;
    res.path = path;

    ONX_Model model;
    ON_TextLog silence;
    if (!model.Read(path.c_str(), &silence)) {
        res.failures.push_back("could not read file");
        return res;
    }

    ONX_ModelComponentIterator it(model, ON_ModelComponent::Type::ModelGeometry);
    for (const ON_ModelComponent* mc = it.FirstComponent();
         mc != nullptr;
         mc = it.NextComponent())
    {
        const ON_ModelGeometryComponent* mgc =
            ON_ModelGeometryComponent::Cast(mc);
        if (!mgc) continue;

        const ON_Geometry* geom = mgc->Geometry(nullptr);
        if (!geom) continue;

        if (geom->ObjectType() == ON::brep_object) {
            const ON_Brep* brep = ON_Brep::Cast(geom);
            ++res.breps_tested;
            std::string reason;
            if (RoundTripBrep(brep, reason)) {
                ++res.breps_passed;
            } else {
                char buf[256];
                std::snprintf(buf, sizeof(buf), "brep #%d: %s",
                              res.breps_tested, reason.c_str());
                res.failures.push_back(buf);
            }
        } else if (geom->ObjectType() == ON::extrusion_object) {
            const ON_Extrusion* ext = ON_Extrusion::Cast(geom);
            if (ext) {
                ON_Brep* b = ext->BrepForm();
                if (b) {
                    ++res.breps_tested;
                    std::string reason;
                    if (RoundTripBrep(b, reason)) {
                        ++res.breps_passed;
                    } else {
                        char buf[256];
                        std::snprintf(buf, sizeof(buf), "extrusion-as-brep #%d: %s",
                                      res.breps_tested, reason.c_str());
                        res.failures.push_back(buf);
                    }
                    delete b;
                } else {
                    ++res.non_brep_objects;
                }
            } else {
                ++res.non_brep_objects;
            }
        } else {
            ++res.non_brep_objects;
        }
    }

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
// Try to decompress a .3dm.gz file to a temporary .3dm file.
// Returns the path to the decompressed file, or empty string on failure.
// ---------------------------------------------------------------------------
static std::string DecompressGz(const std::string& gz_path)
{
    // Build a temporary filename derived from the gz path
    std::string tmp = gz_path.substr(0, gz_path.size() - 3); // strip ".gz"
    // Ensure the tmp path is in /tmp to avoid modifying the source tree
    auto slash = tmp.rfind('/');
    std::string basename = (slash == std::string::npos) ? tmp : tmp.substr(slash + 1);
    std::string tmp_path = std::string("/tmp/") + basename;

    // Use gzip to decompress
    std::string cmd = "gzip -d -c \"" + gz_path + "\" > \"" + tmp_path + "\" 2>/dev/null";
    if (std::system(cmd.c_str()) != 0) {
        return "";
    }
    return tmp_path;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::printf("Usage: %s <path-to-3dm-dir>\n", argv[0]);
        return 1;
    }

    std::string base = argv[1];

    // Collect .3dm files from a flat directory (no version subdirs).
    // Also collect .3dm.gz files and decompress them on-the-fly.
    std::vector<std::string> files;
    std::vector<std::string> tmp_files; // decompressed files to delete later

    ON_FileIterator fi;
    if (fi.Initialize(base.c_str(), nullptr)) {
        for (bool ok = fi.FirstItem(); ok; ok = fi.NextItem()) {
            if (fi.CurrentItemIsDirectory()) continue;
            ON_String full(fi.CurrentItemFullPathName());
            std::string path(full.Array());

            if (EndsWithCI(path, ".3dm")) {
                files.push_back(path);
            } else if (EndsWithCI(path, ".3dm.gz")) {
                std::string decompressed = DecompressGz(path);
                if (!decompressed.empty()) {
                    files.push_back(decompressed);
                    tmp_files.push_back(decompressed);
                    std::printf("INFO: decompressed %s → %s\n",
                                path.c_str(), decompressed.c_str());
                } else {
                    std::printf("WARN: could not decompress %s (skipping)\n",
                                path.c_str());
                }
            }
        }
    }

    if (files.empty()) {
        std::printf("No .3dm files found in: %s\n", base.c_str());
        // Clean up any temp files
        for (const auto& t : tmp_files) std::remove(t.c_str());
        return 1;
    }

    std::printf("Found %d .3dm files\n\n", (int)files.size());

    int total_files   = 0;
    int files_all_pass = 0;
    int files_partial  = 0;
    int files_no_brep  = 0;
    int total_breps   = 0;
    int passed_breps  = 0;

    for (const auto& f : files) {
        ++total_files;
        FileResult r = TestFile(f);

        std::string label = f;
        auto pos = label.rfind('/');
        if (pos != std::string::npos) label = label.substr(pos + 1);

        if (r.breps_tested == 0) {
            ++files_no_brep;
            std::printf("SKIP [%s]: no B-Rep objects (%d other objects)\n",
                        label.c_str(), r.non_brep_objects);
        } else {
            total_breps  += r.breps_tested;
            passed_breps += r.breps_passed;

            if (r.breps_passed == r.breps_tested) {
                ++files_all_pass;
                std::printf("PASS [%s]: %d/%d breps ok, %d other objects\n",
                            label.c_str(),
                            r.breps_passed, r.breps_tested,
                            r.non_brep_objects);
            } else {
                ++files_partial;
                std::printf("FAIL [%s]: %d/%d breps ok, %d other objects\n",
                            label.c_str(),
                            r.breps_passed, r.breps_tested,
                            r.non_brep_objects);
                for (const auto& fail : r.failures)
                    std::printf("       -> %s\n", fail.c_str());
            }
        }
    }

    std::printf("\n=== Summary ===\n");
    std::printf("Files:        %d total, %d all-pass, %d partial, %d no-brep\n",
                total_files, files_all_pass, files_partial, files_no_brep);
    std::printf("B-Reps:       %d/%d passed\n", passed_breps, total_breps);

    // Clean up decompressed temporary files
    for (const auto& t : tmp_files) std::remove(t.c_str());

    return (files_partial == 0) ? 0 : 1;
}
