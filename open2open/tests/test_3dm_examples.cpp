// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_3dm_examples.cpp — Round-trip test for all example .3dm files.
//
// For each .3dm file in opennurbs-8.24.25281.15001/example_files/:
//   1. Read the file with ONX_Model.
//   2. For every ON_Brep object in the model, run the full round-trip:
//        ON_Brep  →  TopoDS_Shape  →  ON_Brep
//   3. Report PASS / FAIL / SKIP (non-brep geometry).
//
// Usage:
//   test_3dm_examples <path-to-example_files-dir>
//
// Exit code: 0 if all tested B-Reps round-trip successfully, 1 otherwise.

#include "open2open/brep_convert.h"

#include "opennurbs.h"

#include <BRepCheck_Analyzer.hxx>
#include <TopoDS_Shape.hxx>

#include <cstdio>
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
        // Try a full repair pass
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
// Geometry-type label helper
// ---------------------------------------------------------------------------
static const char* GeomTypeName(const ON_Geometry* g)
{
    if (!g) return "null";
    switch (g->ObjectType()) {
        case ON::curve_object:   return "curve";
        case ON::surface_object: return "surface";
        case ON::brep_object:    return "brep";
        case ON::mesh_object:    return "mesh";
        case ON::point_object:   return "point";
        case ON::pointset_object:return "pointset";
        case ON::extrusion_object: return "extrusion";
        case ON::subd_object:    return "subd";
        default:                 return "other";
    }
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
    ON_TextLog silence; // suppress opennurbs read error output
    if (!model.Read(path.c_str(), &silence)) {
        res.failures.push_back("could not read file");
        return res;
    }

    // Iterate model geometry objects
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
            // Extrusions can be converted to breps
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
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::printf("Usage: %s <path-to-example_files-dir>\n", argv[0]);
        return 1;
    }

    std::string base = argv[1];

    // Build list of .3dm files by scanning version subdirectories
    std::vector<std::string> files;
    const char* versions[] = {"V1","V2","V3","V4","V5","V6","V7","V8"};
    for (const char* ver : versions) {
        std::string dir = base + "/" + ver;
        ON_FileIterator fi;
        if (fi.Initialize(dir.c_str(), "*.3dm")) {
            for (bool ok = fi.FirstItem(); ok; ok = fi.NextItem()) {
                if (fi.CurrentItemIsDirectory()) continue;
                ON_String full(fi.CurrentItemFullPathName());
                files.push_back(std::string(full.Array()));
            }
        }
    }

    if (files.empty()) {
        std::printf("No .3dm files found under: %s\n", base.c_str());
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

        // Strip directory prefix for display
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

    return (files_partial == 0) ? 0 : 1;
}
