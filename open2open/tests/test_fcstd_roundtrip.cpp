// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_fcstd_roundtrip.cpp — Round-trip test for FreeCAD .FCStd files.
//
// FreeCAD .FCStd files are ZIP archives containing OpenCASCADE BREP shapes
// stored as *.brp files (e.g. PartShape.brp, PartShape1.brp, ...).
//
// For each .FCStd file found recursively in the given directory tree:
//   1. Extract the archive to a temporary directory.
//   2. For each *.brp file found inside:
//      a. Read the shape with BRepTools::Read.
//      b. Convert TopoDS_Shape → ON_Brep (OCCTToON_Brep).
//      c. Convert ON_Brep → TopoDS_Shape (ON_BrepToOCCT).
//      d. Validate the round-tripped shape with BRepCheck_Analyzer.
//      e. Compare surface area (and volume for solids) before and after the
//         round-trip to detect silent geometry distortion.
//      Each shape test runs in a forked child process with a 30-second
//      timeout, so pathological shapes (e.g. datum lines with near-infinite
//      parameter ranges) cannot hang the test suite.
//   3. Clean up the temporary directory.
//
// Usage:
//   test_fcstd_roundtrip <path-to-fcstd-dir>
//
// Exit code: 0 if all shapes round-trip successfully, 1 otherwise.

#include "open2open/brep_convert.h"

#include "opennurbs.h"

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS_Shape.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

// Per-shape timeout in seconds.  Protects against BRP files with near-infinite
// parameter ranges (e.g. FreeCAD datum lines/axes) that cause BRepTools::Read
// or the conversion pipeline to loop indefinitely.
static const int kShapeTimeoutSecs = 30;

// ---------------------------------------------------------------------------
// Compute surface area of a shape.  Returns 0.0 if the shape has no faces.
// ---------------------------------------------------------------------------
static double ShapeArea(const TopoDS_Shape& shape)
{
    GProp_GProps props;
    BRepGProp::SurfaceProperties(shape, props);
    return props.Mass();
}

// ---------------------------------------------------------------------------
// Compute volume of a shape.  Returns 0.0 if the shape is not a solid or has
// zero volume (e.g. open shells).
// ---------------------------------------------------------------------------
static double ShapeVolume(const TopoDS_Shape& shape)
{
    // Only attempt volume for solids / closed shells
    {
        TopExp_Explorer ex(shape, TopAbs_SOLID);
        if (!ex.More()) return 0.0;
    }
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape, props);
    double v = props.Mass();
    return (v >= 0.0) ? v : -v; // volume is always non-negative
}

// ---------------------------------------------------------------------------
// Round-trip one OCCT shape: OCCT → ON_Brep → OCCT.
// Performs topology validity check and geometry (area/volume) comparison.
// Returns true on success, fills reason on failure.
// ---------------------------------------------------------------------------
static bool RoundTripShape(const TopoDS_Shape& shape, std::string& reason)
{
    if (shape.IsNull()) { reason = "null input shape"; return false; }

    // Only process shapes that have faces (pure wires or vertices are skipped)
    {
        TopExp_Explorer fex(shape, TopAbs_FACE);
        if (!fex.More()) {
            reason = "input shape has no faces (skipped)";
            return false;
        }
    }

    // Skip unbounded faces (datum planes, reference geometry) — a face with no
    // bounding edges cannot be represented as a valid ON_Brep B-Rep.
    {
        TopExp_Explorer eex(shape, TopAbs_EDGE);
        if (!eex.More()) {
            reason = "input shape has no edges (datum/reference geometry, skipped)";
            return false;
        }
    }

    // Measure geometry before round-trip
    const double area_orig   = ShapeArea(shape);
    const double volume_orig = ShapeVolume(shape);

    const double kTol = 1e-6;

    // Step 1: OCCT → ON_Brep
    ON_Brep on_brep;
    if (!open2open::OCCTToON_Brep(shape, on_brep, kTol)) {
        ON_wString wlog;
        ON_TextLog log(wlog);
        on_brep.IsValid(&log);
        ON_String slog(wlog);
        if (slog.Length() > 0 && slog.Length() < 256) {
            reason = std::string("OCCTToON_Brep returned false: ") + slog.Array();
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

    // Step 3: validate round-trip shape topology
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

    // Step 4: geometry checks — surface area
    if (area_orig > 0.0) {
        const double area_rt  = ShapeArea(rt_shape);
        const double area_rel = std::fabs(area_rt - area_orig) / area_orig;
        // Allow 1% relative error: round-trip NURBS re-parameterization
        // introduces small but unavoidable differences in surface area.
        if (area_rel > 0.01) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "surface area changed by %.4f%% (orig=%.6g, rt=%.6g)",
                          area_rel * 100.0, area_orig, area_rt);
            reason = buf;
            return false;
        }
    }

    // Step 5: geometry checks — volume (solids only)
    if (volume_orig > 0.0) {
        const double vol_rt  = ShapeVolume(rt_shape);
        const double vol_rel = std::fabs(vol_rt - volume_orig) / volume_orig;
        if (vol_rel > 0.01) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "volume changed by %.4f%% (orig=%.6g, rt=%.6g)",
                          vol_rel * 100.0, volume_orig, vol_rt);
            reason = buf;
            return false;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Result for one *.brp shape inside an FCStd archive.
// ---------------------------------------------------------------------------
struct ShapeResult {
    std::string brp_name;   // filename inside archive, e.g. "PartShape3.brp"
    bool passed    = false;
    bool skipped   = false; // no faces or no edges — not a bounded B-Rep, not a failure
    bool timed_out = false; // exceeded kShapeTimeoutSecs — recorded as failure
    std::string reason;
};

// ---------------------------------------------------------------------------
// Wire protocol constants for fork→parent pipe communication.
// ---------------------------------------------------------------------------
// Result byte layout written by child: [passed:u8][skipped:u8][rlen:u32][reason:rlen bytes]
static const uint8_t kResultPassed  = 1;
static const uint8_t kResultSkipped = 2;

// ---------------------------------------------------------------------------
// Pre-scan a BRP file for extreme numeric token lengths that would cause
// strtod/scanf to hang.
//
// The OpenCASCADE BRP text format stores edge parameter ranges as floating-
// point decimal strings.  FreeCAD emits datum-line/axis shapes whose edges
// have parameter ranges like ±2×10^97, which are represented as 100-digit
// decimal literals.  GNU libc's strtod uses an algorithm that takes
// approximately O(n^2) time for n-digit strings, causing BRepTools::Read to
// stall for minutes on such files.
//
// We scan the raw file bytes for any numeric token (contiguous ASCII digits,
// dots, and sign/exponent characters) whose digit-plus-dot run exceeds
// kMaxTokenDigits.  Any such token indicates unbounded datum geometry; the
// shape can be safely skipped before BRepTools::Read is ever called.
// ---------------------------------------------------------------------------
static const int kMaxTokenDigits = 30; // legitimate BRP numbers have ≤ ~20 sig digits

static bool BrpHasExtremeNumbers(const std::string& path)
{
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;

    int  digits = 0;
    bool extreme = false;
    int  ch;
    while ((ch = std::fgetc(f)) != EOF) {
        if (ch >= '0' && ch <= '9') {
            ++digits;
            if (digits > kMaxTokenDigits) { extreme = true; break; }
        } else if (ch == '.' || ch == '-' || ch == '+') {
            // do not reset digit count for sign/decimal point in a number
        } else {
            digits = 0; // non-numeric character — reset
        }
    }
    std::fclose(f);
    return extreme;
}

// ---------------------------------------------------------------------------
// Test one *.brp file in a forked child with a per-shape timeout.
// BRP files with near-infinite edge parameter ranges (datum lines) first go
// through BrpHasExtremeNumbers(); if extreme values are detected the shape is
// skipped immediately without calling BRepTools::Read, avoiding the strtod
// hang.  The fork+timeout provides a safety net for any other pathological
// input not caught by the pre-scan.
// ---------------------------------------------------------------------------
static ShapeResult TestBrpFile(const std::string& brp_path,
                                const std::string& brp_name)
{
    ShapeResult res;
    res.brp_name = brp_name;

    // Fast pre-scan: skip shapes with extreme numeric values that would cause
    // BRepTools::Read (specifically, GNU libc strtod) to hang.
    if (BrpHasExtremeNumbers(brp_path)) {
        res.skipped = true;
        res.reason  = "extreme edge parameter range (datum geometry, skipped)";
        return res;
    }

    // Create a pipe so the child can send the ShapeResult back to the parent.
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        res.reason = "pipe() failed";
        return res;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        res.reason = "fork() failed";
        return res;
    }

    if (pid == 0) {
        // ---- Child process ----
        close(pipefd[0]); // close read end

        // Do the actual shape test
        TopoDS_Shape shape;
        BRep_Builder builder;
        bool passed  = false;
        bool skipped = false;
        std::string reason;

        if (!BRepTools::Read(shape, brp_path.c_str(), builder)) {
            reason = "BRepTools::Read failed";
        } else {
            bool ok = RoundTripShape(shape, reason);
            if (!ok && reason.find("skipped") != std::string::npos) {
                skipped = true;
            } else {
                passed = ok;
            }
        }

        // Write result to pipe
        uint8_t flags = (passed ? kResultPassed : 0) | (skipped ? kResultSkipped : 0);
        uint32_t rlen = (uint32_t)reason.size();
        ssize_t nw;
        nw = write(pipefd[1], &flags, sizeof(flags)); (void)nw;
        nw = write(pipefd[1], &rlen,  sizeof(rlen));  (void)nw;
        if (rlen > 0) { nw = write(pipefd[1], reason.c_str(), rlen); (void)nw; }
        close(pipefd[1]);
        _exit(0);
    }

    // ---- Parent process ----
    close(pipefd[1]); // close write end

    // Wait for child with kShapeTimeoutSecs timeout (poll every 50 ms).
    const int kPollMs  = 50;
    const int kMaxIter = (kShapeTimeoutSecs * 1000) / kPollMs;

    int status = 0;
    bool child_done = false;
    for (int i = 0; i < kMaxIter; ++i) {
        pid_t w = waitpid(pid, &status, WNOHANG);
        if (w == pid) { child_done = true; break; }
        struct timespec ts { 0, kPollMs * 1000000L };
        nanosleep(&ts, nullptr);
    }

    if (!child_done) {
        // Timeout — kill child and collect it
        kill(pid, SIGKILL);
        waitpid(pid, &status, 0);
        close(pipefd[0]);
        res.timed_out = true;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "timed out after %ds (unbounded geometry?)",
                      kShapeTimeoutSecs);
        res.reason = buf;
        return res;
    }

    // Read result from pipe
    uint8_t  flags = 0;
    uint32_t rlen  = 0;
    if (read(pipefd[0], &flags, sizeof(flags)) == (ssize_t)sizeof(flags) &&
        read(pipefd[0], &rlen,  sizeof(rlen))  == (ssize_t)sizeof(rlen)) {
        res.passed  = (flags & kResultPassed)  != 0;
        res.skipped = (flags & kResultSkipped) != 0;
        if (rlen > 0 && rlen < 4096) {
            res.reason.resize(rlen);
            ssize_t nr = read(pipefd[0], &res.reason[0], rlen);
            if (nr < (ssize_t)rlen) res.reason.resize(nr > 0 ? (size_t)nr : 0);
        }
    } else {
        res.reason = "could not read result from child";
    }
    close(pipefd[0]);
    return res;
}

// ---------------------------------------------------------------------------
// Result for one .FCStd file.
// ---------------------------------------------------------------------------
struct FileResult {
    std::string path;
    std::vector<ShapeResult> shapes;
    std::string error; // set if the archive itself could not be processed
};

// ---------------------------------------------------------------------------
// Helper: run a shell command, ignoring the return value.
// Using an explicit return-value variable suppresses the GCC warn_unused_result
// diagnostic that persists even with a (void) cast on std::system().
// ---------------------------------------------------------------------------
static void RunCmd(const std::string& cmd)
{
    int rc = std::system(cmd.c_str());
    (void)rc;
}

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
// Process one .FCStd file: extract, test each .brp, clean up.
// ---------------------------------------------------------------------------
static FileResult TestFCStdFile(const std::string& fcstd_path)
{
    FileResult res;
    res.path = fcstd_path;

    // Create a unique temporary directory for extraction
    char tmp_tmpl[] = "/tmp/fcstd_XXXXXX";
    char* tmp_dir = mkdtemp(tmp_tmpl);
    if (!tmp_dir) {
        res.error = "could not create temp directory";
        return res;
    }
    std::string tmp_str(tmp_dir);

    // Extract the ZIP archive
    std::string cmd = "unzip -q \"" + fcstd_path + "\" -d \"" + tmp_str
                      + "\" 2>/dev/null";
    {
        int rc = std::system(cmd.c_str());
        if (rc != 0) {
            std::string rm_cmd = "rm -rf \"" + tmp_str + "\"";
            RunCmd(rm_cmd);
            res.error = "unzip failed";
            return res;
        }
    }

    // Find all *.brp files in the extracted directory (non-recursive — all
    // FreeCAD shapes are at the top level of the archive).
    std::vector<std::string> brp_files;
    ON_FileIterator fi;
    if (fi.Initialize(tmp_str.c_str(), nullptr)) {
        for (bool ok = fi.FirstItem(); ok; ok = fi.NextItem()) {
            if (fi.CurrentItemIsDirectory()) continue;
            ON_String full(fi.CurrentItemFullPathName());
            std::string fpath(full.Array());
            if (EndsWithCI(fpath, ".brp")) {
                brp_files.push_back(fpath);
            }
        }
    }

    // Sort so results are deterministic
    std::sort(brp_files.begin(), brp_files.end());

    // Test each .brp file
    for (const auto& brp_path : brp_files) {
        std::string brp_name = brp_path;
        auto slash = brp_name.rfind('/');
        if (slash != std::string::npos) brp_name = brp_name.substr(slash + 1);

        res.shapes.push_back(TestBrpFile(brp_path, brp_name));
    }

    // Clean up temporary directory
    std::string rm_cmd = "rm -rf \"" + tmp_str + "\"";
    RunCmd(rm_cmd);

    return res;
}

// ---------------------------------------------------------------------------
// Collect .FCStd files recursively from a directory tree.
// ---------------------------------------------------------------------------
static void CollectFCStdFiles(const std::string& dir,
                               std::vector<std::string>& out)
{
    ON_FileIterator fi;
    if (!fi.Initialize(dir.c_str(), nullptr)) return;

    for (bool ok = fi.FirstItem(); ok; ok = fi.NextItem()) {
        ON_String full(fi.CurrentItemFullPathName());
        std::string path(full.Array());

        if (fi.CurrentItemIsDirectory()) {
            // Skip hidden directories (e.g. .git)
            ON_String name_on(fi.CurrentItemName());
            std::string name(name_on.Array());
            if (!name.empty() && name[0] != '.') {
                CollectFCStdFiles(path, out);
            }
        } else {
            if (EndsWithCI(path, ".fcstd")) {
                out.push_back(path);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::printf("Usage: %s <path-to-fcstd-dir>\n", argv[0]);
        return 1;
    }

    std::string base = argv[1];

    // Collect .FCStd / .fcstd files recursively
    std::vector<std::string> files;
    CollectFCStdFiles(base, files);

    if (files.empty()) {
        std::printf("No .FCStd files found in: %s\n", base.c_str());
        return 1;
    }

    // Sort for deterministic output
    std::sort(files.begin(), files.end());

    std::printf("Found %d .FCStd files\n\n", (int)files.size());

    int total_files  = 0;
    int files_pass   = 0;
    int files_fail   = 0;
    int total_shapes = 0;
    int pass_shapes  = 0;
    int skip_shapes  = 0;
    int timeout_shapes = 0;

    for (const auto& f : files) {
        ++total_files;
        FileResult r = TestFCStdFile(f);

        std::string label = f;
        auto pos = label.rfind('/');
        if (pos != std::string::npos) label = label.substr(pos + 1);

        if (!r.error.empty()) {
            ++files_fail;
            std::printf("FAIL [%s]: %s\n", label.c_str(), r.error.c_str());
            continue;
        }

        if (r.shapes.empty()) {
            ++files_fail;
            std::printf("FAIL [%s]: no .brp shapes found in archive\n",
                        label.c_str());
            continue;
        }

        int file_pass    = 0;
        int file_skip    = 0;
        int file_fail    = 0;
        int file_timeout = 0;
        for (const auto& s : r.shapes) {
            ++total_shapes;
            if (s.skipped) {
                ++skip_shapes;
                ++file_skip;
            } else if (s.timed_out) {
                ++timeout_shapes;
                ++file_timeout;
                ++file_fail;
            } else if (s.passed) {
                ++pass_shapes;
                ++file_pass;
            } else {
                ++file_fail;
            }
        }

        int testable = (int)r.shapes.size() - file_skip;
        if (file_fail == 0) {
            ++files_pass;
            std::printf("PASS [%s]: %d/%d shapes ok, %d skipped (no faces/edges)\n",
                        label.c_str(), file_pass, testable, file_skip);
        } else {
            ++files_fail;
            std::printf("FAIL [%s]: %d/%d shapes ok, %d skipped (no faces/edges)\n",
                        label.c_str(), file_pass, testable, file_skip);
            for (const auto& s : r.shapes) {
                if (!s.passed && !s.skipped) {
                    std::printf("       -> %s: %s\n",
                                s.brp_name.c_str(), s.reason.c_str());
                }
            }
        }
        std::fflush(stdout);
    }

    int testable_shapes = total_shapes - skip_shapes;
    std::printf("\n=== Summary ===\n");
    std::printf("Files:  %d total, %d passed, %d failed\n",
                total_files, files_pass, files_fail);
    std::printf("Shapes: %d/%d passed, %d skipped (no faces/edges), %d timed out\n",
                pass_shapes, testable_shapes, skip_shapes, timeout_shapes);

    return (files_fail == 0) ? 0 : 1;
}
