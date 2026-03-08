// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// step2brep.cpp — Convert STEP files to OCCT .brp format.
//
// Scans a directory tree recursively for .stp/.step files, reads each with
// STEPControl_Reader, extracts individual solid/shell shapes, and writes them
// as OCCT .brp files in a user-specified output directory.
//
// The generated .brp files can be fed directly to test_occ_roundtrip for
// round-trip validation (OCCT→ON_Brep→OCCT) without needing STEP support
// in the main build.
//
// Usage:
//   step2brep <input-dir> <output-dir>
//
// Output naming: for a file  <input-dir>/sub/foo.step  with N shapes the
// output files are  <output-dir>/sub__foo_0.brp, _1.brp, ...
// Path separators in the relative path are replaced by "__" to avoid creating
// deep subdirectory trees in the output.
//
// Exit code: 0 on success, 1 if any file failed to read/transfer.

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Message.hxx>
#include <STEPControl_Reader.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shape.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

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
// Extract individual solid/shell shapes from a compound.
// Returns shapes in a vector; if none found tries to use the compound itself.
// ---------------------------------------------------------------------------
static void ExtractShapes(const TopoDS_Shape& compound,
                          std::vector<TopoDS_Shape>& shapes)
{
    bool found_solid = false;
    for (TopExp_Explorer ex(compound, TopAbs_SOLID); ex.More(); ex.Next()) {
        shapes.push_back(ex.Current());
        found_solid = true;
    }
    if (!found_solid) {
        for (TopExp_Explorer ex(compound, TopAbs_SHELL); ex.More(); ex.Next()) {
            TopExp_Explorer fex(ex.Current(), TopAbs_FACE);
            if (fex.More()) shapes.push_back(ex.Current());
        }
    }
    if (shapes.empty()) {
        TopExp_Explorer fex(compound, TopAbs_FACE);
        if (fex.More()) shapes.push_back(compound);
    }
}

// ---------------------------------------------------------------------------
// Make a safe output basename: replace path separators and spaces.
// ---------------------------------------------------------------------------
static std::string MakeBasename(const std::string& relpath)
{
    // Strip leading slashes
    std::string s = relpath;
    while (!s.empty() && s[0] == '/') s = s.substr(1);

    // Strip common STEP extensions
    std::string lower = s;
    for (char& c : lower) c = (char)tolower((unsigned char)c);
    if (lower.size() >= 5 && lower.substr(lower.size() - 5) == ".step")
        s = s.substr(0, s.size() - 5);
    else if (lower.size() >= 4 && lower.substr(lower.size() - 4) == ".stp")
        s = s.substr(0, s.size() - 4);

    // Replace path separators with __
    for (char& c : s) {
        if (c == '/' || c == '\\') c = '_';
        else if (c == ' ' || c == '\t') c = '_';
    }
    return s;
}

// ---------------------------------------------------------------------------
// Ensure a directory exists (mkdir -p equivalent, single level).
// ---------------------------------------------------------------------------
static bool EnsureDir(const std::string& path)
{
    struct stat st;
    if (stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode)) return true;
    return mkdir(path.c_str(), 0755) == 0;
}

// ---------------------------------------------------------------------------
// Convert one STEP file; return number of .brp files written, -1 on error.
// ---------------------------------------------------------------------------
static int ConvertStepFile(const std::string& path,
                           const std::string& relpath,
                           const std::string& outdir)
{
    STEPControl_Reader reader;
    // Suppress OCCT console messages
    Message::DefaultMessenger()->ChangePrinters().Clear();

    IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
    if (status != IFSelect_RetDone) {
        fprintf(stderr, "  ERROR: ReadFile failed (status=%d)\n", (int)status);
        return -1;
    }

    Standard_Integer nroots = reader.TransferRoots();
    if (nroots <= 0) {
        fprintf(stderr, "  ERROR: no transferable roots\n");
        return -1;
    }

    TopoDS_Shape compound = reader.OneShape();
    if (compound.IsNull()) {
        fprintf(stderr, "  ERROR: OneShape() returned null\n");
        return -1;
    }

    std::vector<TopoDS_Shape> shapes;
    ExtractShapes(compound, shapes);

    if (shapes.empty()) {
        fprintf(stdout, "  SKIP: no solid/shell shapes extracted\n");
        return 0;
    }

    std::string base = MakeBasename(relpath);
    int written = 0;

    for (int i = 0; i < (int)shapes.size(); ++i) {
        const TopoDS_Shape& shape = shapes[i];
        if (shape.IsNull()) continue;

        // Verify shape has faces
        {
            TopExp_Explorer fex(shape, TopAbs_FACE);
            if (!fex.More()) continue;
        }

        char suffix[64];
        snprintf(suffix, sizeof(suffix), "_%d.brep", i);
        std::string outpath = outdir + "/" + base + suffix;

        if (!BRepTools::Write(shape, outpath.c_str())) {
            fprintf(stderr, "  WARNING: failed to write %s\n", outpath.c_str());
            continue;
        }
        fprintf(stdout, "  wrote %s\n", outpath.c_str());
        ++written;
    }

    return written;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc < 3) {
        fprintf(stderr,
                "Usage: %s <input-dir> <output-dir>\n"
                "\n"
                "  Scans <input-dir> recursively for .stp/.step files,\n"
                "  reads each with OCCT STEPControl_Reader, and writes the\n"
                "  extracted solid/shell shapes as .brp files under <output-dir>.\n"
                "\n"
                "  The .brp files can be used as input to test_occ_roundtrip.\n",
                argv[0]);
        return 1;
    }

    std::string indir  = argv[1];
    std::string outdir = argv[2];

    if (!EnsureDir(outdir)) {
        fprintf(stderr, "ERROR: cannot create output directory: %s\n",
                outdir.c_str());
        return 1;
    }

    std::vector<std::string> files;
    CollectStepFiles(indir, files);

    if (files.empty()) {
        fprintf(stdout, "No .stp/.step files found in %s\n", indir.c_str());
        return 0;
    }

    fprintf(stdout, "Found %d STEP file(s) in %s\n\n",
            (int)files.size(), indir.c_str());

    int total_files  = (int)files.size();
    int ok_files     = 0;
    int failed_files = 0;
    int total_brp    = 0;

    for (const std::string& f : files) {
        // Compute relative path for display and output naming
        std::string relpath = f;
        if (relpath.find(indir) == 0) relpath = relpath.substr(indir.size());
        while (!relpath.empty() && relpath[0] == '/') relpath = relpath.substr(1);

        fprintf(stdout, "[%s]\n", relpath.c_str());
        fflush(stdout);

        int n = ConvertStepFile(f, relpath, outdir);
        if (n < 0) {
            failed_files++;
        } else {
            ok_files++;
            total_brp += n;
        }
        fflush(stdout);
    }

    fprintf(stdout, "\n=== Summary ===\n");
    fprintf(stdout, "Files:  %d total, %d ok, %d failed\n",
            total_files, ok_files, failed_files);
    fprintf(stdout, "Shapes: %d .brp file(s) written to %s\n",
            total_brp, outdir.c_str());

    return (failed_files > 0) ? 1 : 0;
}
