// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_step_roundtrip.cpp — Round-trip test for STEP files.
//
// For each .stp/.step file found recursively in the given directory tree:
//   1. Read the file with STEPControl_Reader.
//   2. For each solid/shell shape extracted from the STEP file:
//      a. Convert TopoDS_Shape → ON_Brep  (OCCTToON_Brep).
//      b. Convert ON_Brep → TopoDS_Shape  (ON_BrepToOCCT).
//      c. Validate the round-tripped shape with BRepCheck_Analyzer.
//      d. Compare surface area (and volume for solids) before and after.
//      Each shape test runs in a forked child process with a 30-second
//      timeout so pathological files cannot hang the test suite.
//   3. Report per-file pass/fail statistics.
//
// Usage:
//   test_step_roundtrip <path-to-step-dir>
//
// Exit code: 0 if all shapes round-trip successfully, 1 otherwise.

#include "open2open/brep_convert.h"

#include "opennurbs.h"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <STEPControl_Reader.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Message.hxx>
#include <Message_ProgressRange.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <dirent.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Collect all .stp/.step files recursively under a directory.
// ---------------------------------------------------------------------------
static void CollectStepFiles(const std::string& dir,
                             std::vector<std::string>& out)
{
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    struct dirent* ent;
    while ((ent = readdir(d)) != nullptr) {
        if (ent->d_name[0] == '.') continue;
        std::string path = dir + "/" + ent->d_name;
        struct stat st;
        if (stat(path.c_str(), &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) {
            CollectStepFiles(path, out);
        } else if (S_ISREG(st.st_mode)) {
            std::string name = ent->d_name;
            // Lower-case extension check
            std::string lower = name;
            for (char& c : lower) c = (char)tolower((unsigned char)c);
            if (lower.size() >= 4) {
                std::string ext4 = lower.substr(lower.size() - 4);
                std::string ext5 = (lower.size() >= 5)
                                       ? lower.substr(lower.size() - 5) : "";
                if (ext4 == ".stp" || ext5 == ".step") {
                    out.push_back(path);
                }
            }
        }
    }
    closedir(d);
    std::sort(out.begin(), out.end());
}

// ---------------------------------------------------------------------------
// Extract all solid/shell shapes from a STEP compound.
// ---------------------------------------------------------------------------
static void ExtractBreps(const TopoDS_Shape& compound,
                         std::vector<TopoDS_Shape>& shapes)
{
    // Try to get individual shells/solids from the compound.
    // TopAbs_SOLID first, then TopAbs_SHELL for open surfaces.
    bool found_solid = false;
    for (TopExp_Explorer ex(compound, TopAbs_SOLID); ex.More(); ex.Next()) {
        shapes.push_back(ex.Current());
        found_solid = true;
    }
    if (!found_solid) {
        for (TopExp_Explorer ex(compound, TopAbs_SHELL); ex.More(); ex.Next()) {
            // Only include shells that have faces
            TopExp_Explorer fex(ex.Current(), TopAbs_FACE);
            if (fex.More()) shapes.push_back(ex.Current());
        }
    }
    // Fallback: use the whole compound if it has faces
    if (shapes.empty()) {
        TopExp_Explorer fex(compound, TopAbs_FACE);
        if (fex.More()) shapes.push_back(compound);
    }
}

// ---------------------------------------------------------------------------
// Wire protocol constants for fork→parent pipe communication.
// ---------------------------------------------------------------------------
enum ShapeFlags : uint32_t {
    FLAG_OK          = 0x01,
    FLAG_SKIP        = 0x02,   // no faces/edges — not a B-Rep
    FLAG_AREA_MISMATCH = 0x04,
    FLAG_VOL_MISMATCH  = 0x08,
};

struct ShapeResult {
    uint32_t    flags   = 0;
    char        reason[512] = {};
};

// ---------------------------------------------------------------------------
// Test one shape. Called in a child process.
// ---------------------------------------------------------------------------
static ShapeResult TestOneShape(const TopoDS_Shape& shape)
{
    ShapeResult res;

    if (shape.IsNull()) {
        res.flags = 0;
        snprintf(res.reason, sizeof(res.reason), "null shape");
        return res;
    }

    // Check for faces
    {
        TopExp_Explorer fex(shape, TopAbs_FACE);
        if (!fex.More()) {
            res.flags = FLAG_SKIP;
            snprintf(res.reason, sizeof(res.reason), "no faces");
            return res;
        }
    }

    // Compute original properties
    GProp_GProps aProps;
    BRepGProp::SurfaceProperties(shape, aProps);
    double orig_area = aProps.Mass();
    double orig_vol  = 0.0;
    bool   is_solid  = false;
    {
        TopExp_Explorer sex(shape, TopAbs_SOLID);
        if (sex.More()) {
            is_solid = true;
            GProp_GProps vProps;
            BRepGProp::VolumeProperties(shape, vProps);
            orig_vol = vProps.Mass();
        }
    }

    const double kTol = 1e-6;

    // Step 1: OCCT → ON_Brep
    ON_Brep on_brep;
    if (!open2open::OCCTToON_Brep(shape, on_brep, kTol)) {
        ON_wString wlog;
        ON_TextLog log(wlog);
        on_brep.IsValid(&log);
        ON_String slog(wlog);
        res.flags = 0;
        if (slog.Length() > 0 && slog.Length() < 480) {
            snprintf(res.reason, sizeof(res.reason),
                     "OCCTToON_Brep returned false: %s", slog.Array());
        } else {
            snprintf(res.reason, sizeof(res.reason),
                     "OCCTToON_Brep returned false");
        }
        return res;
    }
    if (!on_brep.IsValid()) {
        res.flags = 0;
        snprintf(res.reason, sizeof(res.reason),
                 "intermediate ON_Brep is not valid");
        return res;
    }

    // Step 2: ON_Brep → OCCT
    TopoDS_Shape rt = open2open::ON_BrepToOCCT(on_brep, kTol);
    if (rt.IsNull()) {
        res.flags = 0;
        snprintf(res.reason, sizeof(res.reason),
                 "ON_BrepToOCCT returned null");
        return res;
    }

    // Step 3: validate
    BRepCheck_Analyzer bca(rt, /*GeomControls=*/Standard_False);
    if (!bca.IsValid()) {
        res.flags = 0;
        snprintf(res.reason, sizeof(res.reason),
                 "round-tripped shape is not valid (BRepCheck)");
        return res;
    }

    // Step 4: area/volume comparison
    GProp_GProps rtProps;
    BRepGProp::SurfaceProperties(rt, rtProps);
    double rt_area = rtProps.Mass();
    if (orig_area > 1e-12) {
        double pct = fabs(rt_area - orig_area) / orig_area * 100.0;
        if (pct > 1.0) {
            res.flags = FLAG_AREA_MISMATCH;
            snprintf(res.reason, sizeof(res.reason),
                     "surface area changed by %.4g%% (orig=%.6g, rt=%.6g)",
                     pct, orig_area, rt_area);
            return res;
        }
    }
    if (is_solid && fabs(orig_vol) > 1e-12) {
        GProp_GProps vProps2;
        BRepGProp::VolumeProperties(rt, vProps2);
        double rt_vol = vProps2.Mass();
        double pct = fabs(rt_vol - orig_vol) / fabs(orig_vol) * 100.0;
        if (pct > 1.0) {
            res.flags = FLAG_VOL_MISMATCH;
            snprintf(res.reason, sizeof(res.reason),
                     "volume changed by %.4g%% (orig=%.6g, rt=%.6g)",
                     pct, orig_vol, rt_vol);
            return res;
        }
    }

    res.flags = FLAG_OK;
    return res;
}

// ---------------------------------------------------------------------------
// Run TestOneShape in a forked child with timeout.
// ---------------------------------------------------------------------------
static ShapeResult TestOneShapeForked(const TopoDS_Shape& shape,
                                      int timeout_seconds = 30)
{
    ShapeResult res;
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        res.flags = 0;
        snprintf(res.reason, sizeof(res.reason), "pipe() failed");
        return res;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        res.flags = 0;
        snprintf(res.reason, sizeof(res.reason), "fork() failed");
        return res;
    }

    if (pid == 0) {
        // Child
        close(pipefd[0]);
        // Suppress OCCT messages by clearing all printers
        Message::DefaultMessenger()->ChangePrinters().Clear();
        ShapeResult r = TestOneShape(shape);
        ssize_t nw;
        nw = write(pipefd[1], &r.flags, sizeof(r.flags));   (void)nw;
        nw = write(pipefd[1], r.reason, sizeof(r.reason));  (void)nw;
        close(pipefd[1]);
        _exit(0);
    }

    // Parent
    close(pipefd[1]);

    // Set up alarm for timeout
    alarm((unsigned)timeout_seconds);
    ssize_t nr;
    nr = read(pipefd[0], &res.flags,  sizeof(res.flags));
    nr = read(pipefd[0], res.reason,  sizeof(res.reason)); (void)nr;
    close(pipefd[0]);
    alarm(0);

    int status = 0;
    waitpid(pid, &status, 0);
    if (WIFSIGNALED(status)) {
        int sig = WTERMSIG(status);
        if (sig == SIGALRM || sig == SIGKILL) {
            res.flags = 0;
            snprintf(res.reason, sizeof(res.reason), "timed out");
        } else {
            res.flags = 0;
            snprintf(res.reason, sizeof(res.reason),
                     "child killed by signal %d", sig);
        }
    }
    return res;
}

// ---------------------------------------------------------------------------
// File-level result
// ---------------------------------------------------------------------------
struct FileResult {
    std::string path;
    int total    = 0;
    int passed   = 0;
    int skipped  = 0;
    int timedout = 0;
    std::vector<std::string> failures;
};

// ---------------------------------------------------------------------------
// Test one STEP file.
// ---------------------------------------------------------------------------
static FileResult TestStepFile(const std::string& path)
{
    FileResult fr;
    fr.path = path;

    // Read with STEPControl_Reader
    STEPControl_Reader reader;
    // Suppress OCCT messages by clearing all printers
    Message::DefaultMessenger()->ChangePrinters().Clear();

    IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
    if (status != IFSelect_RetDone) {
        fr.failures.push_back("STEPControl_Reader::ReadFile failed (status="
                              + std::to_string((int)status) + ")");
        return fr;
    }

    // Transfer all roots
    Standard_Integer nroots = reader.TransferRoots();
    if (nroots <= 0) {
        fr.failures.push_back("no transferable roots in STEP file");
        return fr;
    }

    // Get the combined shape
    TopoDS_Shape compound = reader.OneShape();
    if (compound.IsNull()) {
        fr.failures.push_back("OneShape() returned null");
        return fr;
    }

    // Extract individual solids/shells for testing
    std::vector<TopoDS_Shape> shapes;
    ExtractBreps(compound, shapes);

    if (shapes.empty()) {
        fr.skipped++;
        return fr;
    }

    for (const TopoDS_Shape& shape : shapes) {
        fr.total++;
        ShapeResult res = TestOneShapeForked(shape);
        if (res.flags & FLAG_SKIP) {
            fr.skipped++;
            fr.total--;
        } else if (res.flags & FLAG_OK) {
            fr.passed++;
        } else {
            std::string reason = res.reason;
            if (res.flags == 0 && strncmp(res.reason, "timed out", 9) == 0) {
                fr.timedout++;
                fr.failures.push_back("shape #" + std::to_string(fr.total)
                                      + ": timed out");
            } else {
                fr.failures.push_back("shape #" + std::to_string(fr.total)
                                      + ": " + reason);
            }
        }
    }
    return fr;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <step-dir>\n", argv[0]);
        return 1;
    }

    std::string dir = argv[1];
    std::vector<std::string> files;
    CollectStepFiles(dir, files);

    if (files.empty()) {
        fprintf(stdout, "No .stp/.step files found in %s\n", dir.c_str());
        return 0;
    }

    fprintf(stdout, "Found %d .stp/.step files\n\n", (int)files.size());
    fflush(stdout);

    int total_files   = (int)files.size();
    int passed_files  = 0;
    int failed_files  = 0;
    int total_shapes  = 0;
    int passed_shapes = 0;
    int skipped_shapes = 0;
    int timedout_shapes = 0;

    for (const std::string& f : files) {
        // Print relative path
        std::string display = f;
        if (display.find(dir) == 0) display = display.substr(dir.size());
        while (!display.empty() && display[0] == '/') display = display.substr(1);

        FileResult fr = TestStepFile(f);
        total_shapes   += fr.total;
        passed_shapes  += fr.passed;
        skipped_shapes += fr.skipped;
        timedout_shapes += fr.timedout;

        bool all_passed = fr.failures.empty()
                          && fr.timedout == 0
                          && fr.total > 0;
        bool is_empty   = (fr.total == 0 && fr.skipped == 0
                           && fr.failures.empty());

        if (is_empty) {
            fprintf(stdout, "SKIP [%s]: no shapes extracted\n",
                    display.c_str());
            passed_files++;  // don't penalize empty files
        } else if (all_passed) {
            fprintf(stdout, "PASS [%s]: %d/%d shapes ok, %d skipped\n",
                    display.c_str(), fr.passed, fr.total, fr.skipped);
            passed_files++;
        } else {
            fprintf(stdout, "FAIL [%s]: %d/%d shapes ok, %d skipped\n",
                    display.c_str(), fr.passed, fr.total, fr.skipped);
            for (const std::string& r : fr.failures)
                fprintf(stdout, "       -> %s\n", r.c_str());
            failed_files++;
        }
        fflush(stdout);
    }

    fprintf(stdout, "\n=== Summary ===\n");
    fprintf(stdout, "Files:  %d total, %d passed, %d failed\n",
            total_files, passed_files, failed_files);
    fprintf(stdout, "Shapes: %d/%d passed, %d skipped, %d timed out\n",
            passed_shapes, total_shapes, skipped_shapes, timedout_shapes);

    return (failed_files > 0) ? 1 : 0;
}
