// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// test_mesh.cpp — Unit tests for ON_Mesh ↔ Poly_Triangulation conversion.
//
// Tests cover:
//   1. Single-triangle round-trip (positions preserved).
//   2. Single-quad triangulation (quad becomes two OCCT triangles; topology
//      is verified after the reverse conversion).
//   3. Per-vertex normals round-trip.
//   4. Per-vertex UV texture coordinates round-trip.
//   5. Empty / degenerate mesh handling (null handle returned gracefully).

#include "open2open/mesh_convert.h"

#include "opennurbs.h"

#include <Poly_Triangulation.hxx>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// Tolerance used for floating-point comparisons.
// ON_Mesh stores positions as single-precision floats so 1e-5 is appropriate.
// ---------------------------------------------------------------------------
static const double kTol = 1e-5;

static bool near(double a, double b, double tol = kTol)
{
    return std::fabs(a - b) <= tol;
}

// ---------------------------------------------------------------------------
// Helper: build a minimal two-triangle mesh (a unit square split on the
// diagonal) with known vertex positions.
//
//   v0(0,0,0) --- v1(1,0,0)
//       |        /  |
//       |      /    |
//   v3(0,1,0)--- v2(1,1,0)
//
// Faces: tri0={0,1,2}, tri1={0,2,3}
// ---------------------------------------------------------------------------
static ON_Mesh BuildSquareMesh()
{
    ON_Mesh mesh;
    mesh.m_V.SetCapacity(4);
    mesh.m_V.SetCount(4);
    mesh.m_V[0] = ON_3fPoint(0.0f, 0.0f, 0.0f);
    mesh.m_V[1] = ON_3fPoint(1.0f, 0.0f, 0.0f);
    mesh.m_V[2] = ON_3fPoint(1.0f, 1.0f, 0.0f);
    mesh.m_V[3] = ON_3fPoint(0.0f, 1.0f, 0.0f);

    mesh.m_F.SetCapacity(2);
    mesh.m_F.SetCount(2);
    // triangle: vi[2] == vi[3]
    mesh.m_F[0].vi[0] = 0; mesh.m_F[0].vi[1] = 1;
    mesh.m_F[0].vi[2] = 2; mesh.m_F[0].vi[3] = 2;
    mesh.m_F[1].vi[0] = 0; mesh.m_F[1].vi[1] = 2;
    mesh.m_F[1].vi[2] = 3; mesh.m_F[1].vi[3] = 3;
    return mesh;
}

// ===========================================================================
// Test 1 — Simple triangle round-trip
// ===========================================================================
static bool TestTriangleRoundTrip()
{
    ON_Mesh src;
    src.m_V.SetCapacity(3);
    src.m_V.SetCount(3);
    src.m_V[0] = ON_3fPoint(0.0f, 0.0f, 0.0f);
    src.m_V[1] = ON_3fPoint(1.0f, 0.0f, 0.0f);
    src.m_V[2] = ON_3fPoint(0.5f, 1.0f, 0.0f);

    src.m_F.SetCapacity(1);
    src.m_F.SetCount(1);
    src.m_F[0].vi[0] = 0; src.m_F[0].vi[1] = 1;
    src.m_F[0].vi[2] = 2; src.m_F[0].vi[3] = 2; // triangle encoding

    Handle(Poly_Triangulation) tri = open2open::ON_MeshToOCCT(src);
    if (tri.IsNull()) {
        std::fprintf(stderr, "FAIL TestTriangleRoundTrip: ON_MeshToOCCT returned null\n");
        return false;
    }
    if (tri->NbNodes() != 3 || tri->NbTriangles() != 1) {
        std::fprintf(stderr,
                     "FAIL TestTriangleRoundTrip: expected 3 nodes / 1 tri, got %d/%d\n",
                     tri->NbNodes(), tri->NbTriangles());
        return false;
    }

    ON_Mesh dst;
    if (!open2open::OCCTToON_Mesh(tri, dst)) {
        std::fprintf(stderr, "FAIL TestTriangleRoundTrip: OCCTToON_Mesh failed\n");
        return false;
    }

    // Vertex positions round-trip (single-precision tolerance).
    for (int i = 0; i < 3; ++i) {
        if (!near(dst.m_V[i].x, src.m_V[i].x) ||
            !near(dst.m_V[i].y, src.m_V[i].y) ||
            !near(dst.m_V[i].z, src.m_V[i].z)) {
            std::fprintf(stderr,
                         "FAIL TestTriangleRoundTrip: vertex %d mismatch\n", i);
            return false;
        }
    }

    // Face topology preserved.
    const ON_MeshFace& f = dst.m_F[0];
    if (f.vi[0] != 0 || f.vi[1] != 1 || f.vi[2] != 2 || f.vi[3] != 2) {
        std::fprintf(stderr, "FAIL TestTriangleRoundTrip: face topology mismatch\n");
        return false;
    }

    std::puts("PASS TestTriangleRoundTrip");
    return true;
}

// ===========================================================================
// Test 2 — Quad triangulation: one quad becomes two OCCT triangles.
// ===========================================================================
static bool TestQuadTriangulation()
{
    ON_Mesh src;
    src.m_V.SetCapacity(4);
    src.m_V.SetCount(4);
    src.m_V[0] = ON_3fPoint(0.0f, 0.0f, 0.0f);
    src.m_V[1] = ON_3fPoint(1.0f, 0.0f, 0.0f);
    src.m_V[2] = ON_3fPoint(1.0f, 1.0f, 0.0f);
    src.m_V[3] = ON_3fPoint(0.0f, 1.0f, 0.0f);

    src.m_F.SetCapacity(1);
    src.m_F.SetCount(1);
    // quad: all four vi[] distinct
    src.m_F[0].vi[0] = 0; src.m_F[0].vi[1] = 1;
    src.m_F[0].vi[2] = 2; src.m_F[0].vi[3] = 3;

    Handle(Poly_Triangulation) tri = open2open::ON_MeshToOCCT(src);
    if (tri.IsNull()) {
        std::fprintf(stderr, "FAIL TestQuadTriangulation: ON_MeshToOCCT returned null\n");
        return false;
    }
    // One quad → two OCCT triangles.
    if (tri->NbTriangles() != 2) {
        std::fprintf(stderr,
                     "FAIL TestQuadTriangulation: expected 2 triangles, got %d\n",
                     tri->NbTriangles());
        return false;
    }

    // After round-tripping back, we get two triangle faces.
    ON_Mesh dst;
    if (!open2open::OCCTToON_Mesh(tri, dst)) {
        std::fprintf(stderr, "FAIL TestQuadTriangulation: OCCTToON_Mesh failed\n");
        return false;
    }
    if (dst.m_F.Count() != 2) {
        std::fprintf(stderr,
                     "FAIL TestQuadTriangulation: expected 2 ON faces, got %d\n",
                     dst.m_F.Count());
        return false;
    }
    // Both result faces should be triangles (vi[2] == vi[3]).
    for (int fi = 0; fi < 2; ++fi) {
        if (!dst.m_F[fi].IsTriangle()) {
            std::fprintf(stderr,
                         "FAIL TestQuadTriangulation: face %d is not a triangle\n", fi);
            return false;
        }
    }

    std::puts("PASS TestQuadTriangulation");
    return true;
}

// ===========================================================================
// Test 3 — Per-vertex normals round-trip.
// ===========================================================================
static bool TestNormalsRoundTrip()
{
    ON_Mesh src = BuildSquareMesh();

    // Assign upward-facing normals to all vertices.
    src.m_N.SetCapacity(4);
    src.m_N.SetCount(4);
    for (int i = 0; i < 4; ++i)
        src.m_N[i] = ON_3fVector(0.0f, 0.0f, 1.0f);

    Handle(Poly_Triangulation) tri = open2open::ON_MeshToOCCT(src);
    if (tri.IsNull() || !tri->HasNormals()) {
        std::fprintf(stderr, "FAIL TestNormalsRoundTrip: no normals in OCCT mesh\n");
        return false;
    }

    ON_Mesh dst;
    if (!open2open::OCCTToON_Mesh(tri, dst) || !dst.HasVertexNormals()) {
        std::fprintf(stderr, "FAIL TestNormalsRoundTrip: no normals in round-tripped mesh\n");
        return false;
    }

    for (int i = 0; i < 4; ++i) {
        if (!near(dst.m_N[i].x, 0.0) ||
            !near(dst.m_N[i].y, 0.0) ||
            !near(dst.m_N[i].z, 1.0)) {
            std::fprintf(stderr,
                         "FAIL TestNormalsRoundTrip: normal %d mismatch\n", i);
            return false;
        }
    }

    std::puts("PASS TestNormalsRoundTrip");
    return true;
}

// ===========================================================================
// Test 4 — Per-vertex UV texture coordinates round-trip.
// ===========================================================================
static bool TestUVRoundTrip()
{
    ON_Mesh src = BuildSquareMesh();

    // Assign UV coordinates matching the XY positions.
    src.m_T.SetCapacity(4);
    src.m_T.SetCount(4);
    src.m_T[0] = ON_2fPoint(0.0f, 0.0f);
    src.m_T[1] = ON_2fPoint(1.0f, 0.0f);
    src.m_T[2] = ON_2fPoint(1.0f, 1.0f);
    src.m_T[3] = ON_2fPoint(0.0f, 1.0f);

    Handle(Poly_Triangulation) tri = open2open::ON_MeshToOCCT(src);
    if (tri.IsNull() || !tri->HasUVNodes()) {
        std::fprintf(stderr, "FAIL TestUVRoundTrip: no UV in OCCT mesh\n");
        return false;
    }

    ON_Mesh dst;
    if (!open2open::OCCTToON_Mesh(tri, dst) || !dst.HasTextureCoordinates()) {
        std::fprintf(stderr, "FAIL TestUVRoundTrip: no UV in round-tripped mesh\n");
        return false;
    }

    const float expected_u[] = {0.0f, 1.0f, 1.0f, 0.0f};
    const float expected_v[] = {0.0f, 0.0f, 1.0f, 1.0f};
    for (int i = 0; i < 4; ++i) {
        if (!near(dst.m_T[i].x, expected_u[i]) ||
            !near(dst.m_T[i].y, expected_v[i])) {
            std::fprintf(stderr,
                         "FAIL TestUVRoundTrip: UV[%d] mismatch\n", i);
            return false;
        }
    }

    std::puts("PASS TestUVRoundTrip");
    return true;
}

// ===========================================================================
// Test 5 — Double-precision vertex positions are used when available.
// ===========================================================================
static bool TestDoublePrecision()
{
    ON_Mesh src;
    const double x = 1.23456789012345;
    const double y = 9.87654321098765;
    const double z = 3.14159265358979;

    // Populate float array (rounded) and double array (full precision).
    src.m_V.SetCapacity(3);
    src.m_V.SetCount(3);
    src.m_V[0] = ON_3fPoint(static_cast<float>(x),
                             static_cast<float>(y),
                             static_cast<float>(z));
    src.m_V[1] = ON_3fPoint(2.0f, 0.0f, 0.0f);
    src.m_V[2] = ON_3fPoint(0.0f, 2.0f, 0.0f);

    src.m_dV.SetCapacity(3);
    src.m_dV.SetCount(3);
    src.m_dV[0] = ON_3dPoint(x, y, z);
    src.m_dV[1] = ON_3dPoint(2.0, 0.0, 0.0);
    src.m_dV[2] = ON_3dPoint(0.0, 2.0, 0.0);

    src.m_F.SetCapacity(1);
    src.m_F.SetCount(1);
    src.m_F[0].vi[0] = 0; src.m_F[0].vi[1] = 1;
    src.m_F[0].vi[2] = 2; src.m_F[0].vi[3] = 2;

    Handle(Poly_Triangulation) tri = open2open::ON_MeshToOCCT(src);
    if (tri.IsNull()) {
        std::fprintf(stderr, "FAIL TestDoublePrecision: null handle\n");
        return false;
    }

    // The OCCT node should carry the full double-precision value.
    const gp_Pnt& p = tri->Node(1);
    const double tol_dp = 1e-12;
    if (std::fabs(p.X() - x) > tol_dp ||
        std::fabs(p.Y() - y) > tol_dp ||
        std::fabs(p.Z() - z) > tol_dp) {
        std::fprintf(stderr,
                     "FAIL TestDoublePrecision: double precision not preserved\n");
        return false;
    }

    std::puts("PASS TestDoublePrecision");
    return true;
}

// ===========================================================================
// Test 6 — Empty mesh and null handle handled gracefully.
// ===========================================================================
static bool TestEmptyMesh()
{
    // Empty ON_Mesh → null handle.
    ON_Mesh empty;
    Handle(Poly_Triangulation) t = open2open::ON_MeshToOCCT(empty);
    if (!t.IsNull()) {
        std::fprintf(stderr, "FAIL TestEmptyMesh: expected null handle for empty mesh\n");
        return false;
    }

    // Null handle → false.
    ON_Mesh dst;
    if (open2open::OCCTToON_Mesh(Handle(Poly_Triangulation)(), dst)) {
        std::fprintf(stderr, "FAIL TestEmptyMesh: OCCTToON_Mesh should return false for null\n");
        return false;
    }

    std::puts("PASS TestEmptyMesh");
    return true;
}

// ===========================================================================
// main
// ===========================================================================
int main()
{
    int pass = 0, fail = 0;

    auto run = [&](bool (*fn)(), const char* name) {
        (void)name;
        if (fn()) ++pass; else ++fail;
    };

    run(TestTriangleRoundTrip,  "TestTriangleRoundTrip");
    run(TestQuadTriangulation,  "TestQuadTriangulation");
    run(TestNormalsRoundTrip,   "TestNormalsRoundTrip");
    run(TestUVRoundTrip,        "TestUVRoundTrip");
    run(TestDoublePrecision,    "TestDoublePrecision");
    run(TestEmptyMesh,          "TestEmptyMesh");

    std::printf("\n%d/%d mesh tests passed\n", pass, pass + fail);
    return fail > 0 ? 1 : 0;
}
