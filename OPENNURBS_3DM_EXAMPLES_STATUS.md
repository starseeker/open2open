# OpenNURBS 3DM Example Files — Round-Trip Status

Round-trip test: ON_Brep → TopoDS_Shape (OCCT) → ON_Brep, validated with
`ON_Brep::IsValid()`.  Run with:

```
build/tests/test_3dm_examples opennurbs-8.24.25281.15001/example_files
```

## Overall Results

**1177 / 1209 B-Reps pass** across 168 example files (97.4 %).

| Category | Count |
|---|---|
| Files all-pass | 83 |
| Files partial | 14 |
| Files no B-Rep (skip) | 71 |

---

## Fixes Applied This Session

### Seam-edge pcurve domain + face-orientation fix (`brep_convert.cpp`)

Root cause: `UpdateEdge(E, C1, C2, F, tol)` for seam edges was broken in
two independent ways.

1. **Domain mismatch**: `Geom2d_Curve::Reversed()` in OCCT preserves the
   original parameter domain.  The reversed REV-seam pcurve therefore had
   domain `[-t1, -t0]` while `BRep_Builder::Range` stored `[t0, t1]`.
   OCCT silently extrapolated outside the curve domain, giving wildly wrong
   UV values (e.g. v ≈ 28 instead of v ≈ 5 on a surface with v ∈ [5, 18]).

2. **Face-orientation swap**: When a face has `m_bRev = 1`, OCCT swaps which
   of `PCurve()` / `PCurve2()` is returned for FORWARD vs REVERSED edge
   occurrences.  The C1/C2 arguments to `UpdateEdge` must therefore be
   swapped for reversed faces.

Fix: build a fresh `Geom2d_BSplineCurve` with the same poles but knots
linearly remapped to `[sp.t0, sp.t1]`; swap C1/C2 when `f.m_bRev == 1`.

Effect: previously-failing v1/v2/v3 RhinoLogo (0/6), Soccer (16/22),
HumanHead1 (17/22), T-Joint (0/9), SphericalSpiral (0/4), etc. all pass now.

---

## Remaining Failures (32 breps in 14 files)

### Unfixable: structurally invalid input brep
| File | Breps | Details |
|---|---|---|
| v1_Camera.3dm | 2/41 fail | breps #36, #41: `input brep invalid (after tolerance repair)` |
| v2_Camera.3dm | 2/41 fail | breps #1, #6: `input brep invalid (after tolerance repair)` |

These breps are already invalid in the source 3dm file (openNURBS reports
them as invalid before any conversion).  Nothing can be done here.

### Geometric tolerance: seam endpoint mismatch

| File | Breps | Details |
|---|---|---|
| v4_Fluted_Cup.3dm | 0/2 | OCCTToON_Brep: trim endpoint 0.227 from edge (edge tol 7e-5) |
| v4_DVDCase_Solid.3dm | 0/1 | OCCTToON_Brep returned false |
| v4_MechPartA.3dm | 0/1 | OCCTToON_Brep returned false |
| v4_Wheel_PG.3dm | 1/6 | OCCTToON_Brep returned false |
| v5_disk_brake.3dm | 4/66 | OCCTToON_Brep returned false |
| v5_clip.3dm | 4/16 | OCCTToON_Brep returned false |
| v5_teacup.3dm | 2/6 | OCCTToON_Brep returned false |
| v5_teapot.3dm | 1/3 | OCCTToON_Brep returned false |

These models contain surfaces that are closed-but-not-periodic in U (a
valid Rhino construct).  On such surfaces the ON_Brep stores a seam edge
at `U = U_period` where `S(U=a, v) ≡ S(U=a+period, v)` in 3-D.  After the
round-trip the seam trim at the "low-U" side evaluates to a 3-D point that
disagrees with the stored 3-D edge endpoint by a large amount (≥ 0.2 units),
because the two sides of the seam parametrically evaluate to different values
on a non-periodic B-spline even though they are the same physical point.
This causes `ON_Brep::IsValid()` to fail.  Addressing this would require
either (a) rebuilding the edge's 3-D curve to match both seam sides, or
(b) recognising the mismatch and updating the edge tolerance accordingly.

### Camera body: OCCTToON_Brep failure

| File | Breps | Details |
|---|---|---|
| v1_Camera.3dm | 3/41 fail | breps #10, #28, #30 |
| v2_Camera.3dm | 3/41 fail | breps #12, #14, #32 |
| v3_Camera.3dm | 3/41 fail | breps #12, #30, #32 |

Partial failure in a complex mechanical model. The failing breps are likely
surfaces with very high-degree NURBS trim curves that produce numerical issues
in the OCCT wire topology. Investigation of individual breps is needed.

### Rhino logo brep #2 (all versions)

| File | Status |
|---|---|
| v5_rhino_logo.3dm | 5/6 |
| v6_rhino_logo.3dm | 5/6 |
| v7_rhino_logo_nurbs.3dm | 5/6 |

Brep #2 in these files fails OCCTToON_Brep.  The other 5 pass.  Likely a
complex trimmed surface with specific topology not yet handled.

---

## File-by-File Results (version V1 shown; V2/V3 identical unless noted)

| File | V1 Result | V2 | V3 | V4 | V5 | V6 | V7 | V8 |
|---|---|---|---|---|---|---|---|---|
| Camera.3dm | 36/41 ❌ | 36/41 | 38/41 | — | — | — | — | — |
| DeformCubes.3dm | 25/25 ✅ | ✅ | ✅ | — | — | — | — | — |
| DeformSpheres.3dm | 4/4 ✅ | ✅ | ✅ | — | — | — | — | — |
| Flow2.3dm | 3/3 ✅ | ✅ | ✅ | — | — | — | — | — |
| HumanHead1.3dm | 22/22 ✅ | 20/20 ✅ | ✅ | — | — | — | — | — |
| HumanHead2.3dm | 10/10 ✅ | ✅ | ✅ | — | — | — | — | — |
| Lightbulb.3dm | 44/44 ✅ | ✅ | ✅ | — | — | — | — | — |
| MatchSrf.3dm | 15/15 ✅ | ✅ | ✅ | — | — | — | — | — |
| Patch.3dm | 63/63 ✅ | ✅ | ✅ | — | — | — | — | — |
| RhinoLogo.3dm | 6/6 ✅ | ✅ | ✅ | — | 5/6 ❌ | 5/6 ❌ | — | — |
| SketchOnMesh.3dm | 7/7 ✅ | ✅ | ✅ | — | — | — | — | — |
| Smooth.3dm | 4/4 ✅ | ✅ | ✅ | — | — | — | — | — |
| Soccer.3dm | 22/22 ✅ | ✅ | ✅ | — | — | — | — | — |
| Speeder.3dm | 2/2 ✅ | ✅ | ✅ | — | — | — | — | — |
| SphericalSpiral.3dm | 4/4 ✅ | ✅ | ✅ | — | — | — | — | — |
| Stearman.3dm | 1/1 ✅ | ✅ | ✅ | — | — | — | — | — |
| T-Joint1.3dm | 9/9 ✅ | ✅ | ✅ | — | — | — | — | — |
| T-Joint2.3dm | 13/13 ✅ | ✅ | ✅ | — | — | — | — | — |
| Terrain.3dm | 1/1 ✅ | ✅ | ✅ | — | — | — | — | — |
| UntrimmedSurfaces.3dm | 11/11 ✅ | ✅ | ✅ | — | — | — | — | — |
| five_solids.3dm | 5/5 ✅ | — | — | — | — | — | — | — |
| Fluted_Cup.3dm | — | — | — | 0/2 ❌ | — | — | — | — |
| DVDCase_Solid.3dm | — | — | — | 0/1 ❌ | — | — | — | — |
| MechPartA.3dm | — | — | — | 0/1 ❌ | — | — | — | — |
| MechPartB.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| WishBone.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| SolidHandle.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| SaltAndPepper.3dm | — | — | — | 2/2 ✅ | — | — | — | — |
| RhinoPhone.3dm | — | — | — | 17/17 ✅ | — | — | — | — |
| DinerMug.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| PerfumeBottle_Solid.3dm | — | — | — | 2/2 ✅ | — | — | — | — |
| Gear.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| example_file.3dm | — | — | — | 5/5 ✅ | 7/7 ✅ | — | — | — |
| Wheel_PG.3dm | — | — | — | 5/6 ❌ | — | — | — | — |
| TreeFrog.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| Prarie settle.3dm | — | — | — | 101/101 ✅ | — | — | — | — |
| ring.3dm | — | — | — | — | 1/1 ✅ | — | — | — |
| clip.3dm | — | — | — | — | 12/16 ❌ | — | — | — |
| teacup.3dm | — | — | — | — | 4/6 ❌ | — | — | — |
| teapot.3dm | — | — | — | — | 2/3 ❌ | — | — | — |
| disk_brake.3dm | — | — | — | — | 62/66 ❌ | — | — | — |
| extrusions.3dm | — | — | — | — | 7/7 ✅ | — | — | — |
| section_hatch.3dm | — | — | — | — | — | — | — | 3/3 ✅ |
| selective_clipping.3dm | — | — | — | — | — | — | — | 6/6 ✅ |
| my_trimmed_surface.3dm | — | — | — | — | 1/1 ✅ | — | 1/1 ✅ | 1/1 ✅ |
| my_brep.3dm | — | ✅ | ✅ | — | — | — | 1/1 ✅ | — |
| rhino_logo_nurbs.3dm | — | — | — | — | — | — | 5/6 ❌ | — |
