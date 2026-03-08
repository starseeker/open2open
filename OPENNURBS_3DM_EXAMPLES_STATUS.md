# OpenNURBS 3DM Example Files — Round-Trip Status

Round-trip test: ON_Brep → TopoDS_Shape (OCCT) → ON_Brep, validated with
`ON_Brep::IsValid()`.  Run with:

```
build/tests/test_3dm_examples opennurbs-8.24.25281.15001/example_files
```

## Overall Results

**1204 / 1209 B-Reps pass** across 168 example files (99.6 %).

| Category | Count |
|---|---|
| Files all-pass | 94 |
| Files partial | 3 |
| Files no B-Rep (skip) | 71 |

---

## Fixes Applied (cumulative)

### Trim proxy domain: SetProxyCurve instead of SetProxyCurveDomain

Root cause: `ON_BrepTrim` inherits from `ON_CurveProxy`, which has two
separate domain fields:

- `m_this_domain` — used by `trim.Domain()` and `IsValid()`'s loop gap check
- `m_real_curve_domain` — used by `PointAtEnd()` via proxy mapping

`NewTrim()` sets both to the full curve domain.  The old code called
`SetProxyCurveDomain([pc_t0, pc_t1])`, which only updates `m_real_curve_domain`.
When the pcurve is a sub-range of the stored B-Spline, `IsValid()` evaluates
at `m_this_domain[1]` (the wrong, wider endpoint) and reports a false UV gap.

Fix: replace `SetProxyCurveDomain` with the two-arg `SetProxyCurve(c2, dom)`
which sets both domains atomically.  Guards ensure the sub-interval is both
strictly increasing *and* fully contained within the stored curve's domain
(falling back to the full domain when either guard fails).

Effect: NIST_MBE_PMI_7-10.3dm (brep #1) now passes.

### Seam-edge pcurve domain + face-orientation fix

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

## Remaining Failures (5 breps in 3 files)

### Unfixable: structurally invalid input brep

| File | Breps | Details |
|---|---|---|
| v1_Camera.3dm | 2/41 fail | breps #36, #41: `input brep invalid (after tolerance repair)` |
| v2_Camera.3dm | 2/41 fail | breps #1, #6: `input brep invalid (after tolerance repair)` |

These breps are already invalid in the source 3dm file (openNURBS reports
them as invalid before any conversion).  Nothing can be done here.

### RhinoPhone: OCCTToON_Brep failure

| File | Breps | Details |
|---|---|---|
| v4_RhinoPhone.3dm | 16/17 | brep #14: `OCCTToON_Brep returned false` |

Partial failure in a complex mechanical model.  Investigation of the
individual failing brep is needed.

---

## File-by-File Results (version V1 shown; V2/V3 identical unless noted)

| File | V1 Result | V2 | V3 | V4 | V5 | V6 | V7 | V8 |
|---|---|---|---|---|---|---|---|---|
| Camera.3dm | 39/41 ❌ | 39/41 | 41/41 ✅ | — | — | — | — | — |
| DeformCubes.3dm | 25/25 ✅ | ✅ | ✅ | — | — | — | — | — |
| DeformSpheres.3dm | 4/4 ✅ | ✅ | ✅ | — | — | — | — | — |
| Flow2.3dm | 3/3 ✅ | ✅ | ✅ | — | — | — | — | — |
| HumanHead1.3dm | 22/22 ✅ | 20/20 ✅ | ✅ | — | — | — | — | — |
| HumanHead2.3dm | 10/10 ✅ | ✅ | ✅ | — | — | — | — | — |
| Lightbulb.3dm | 44/44 ✅ | ✅ | ✅ | — | — | — | — | — |
| MatchSrf.3dm | 15/15 ✅ | ✅ | ✅ | — | — | — | — | — |
| Patch.3dm | 63/63 ✅ | ✅ | ✅ | — | — | — | — | — |
| RhinoLogo.3dm | 6/6 ✅ | ✅ | ✅ | — | 6/6 ✅ | 6/6 ✅ | — | — |
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
| Fluted_Cup.3dm | — | — | — | 2/2 ✅ | — | — | — | — |
| DVDCase_Solid.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| MechPartA.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| MechPartB.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| WishBone.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| SolidHandle.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| SaltAndPepper.3dm | — | — | — | 2/2 ✅ | — | — | — | — |
| RhinoPhone.3dm | — | — | — | 16/17 ❌ | — | — | — | — |
| DinerMug.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| PerfumeBottle_Solid.3dm | — | — | — | 2/2 ✅ | — | — | — | — |
| Gear.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| example_file.3dm | — | — | — | 5/5 ✅ | 7/7 ✅ | — | — | — |
| Wheel_PG.3dm | — | — | — | 6/6 ✅ | — | — | — | — |
| TreeFrog.3dm | — | — | — | 1/1 ✅ | — | — | — | — |
| Prarie settle.3dm | — | — | — | 101/101 ✅ | — | — | — | — |
| ring.3dm | — | — | — | — | 1/1 ✅ | — | — | — |
| clip.3dm | — | — | — | — | 16/16 ✅ | — | — | — |
| teacup.3dm | — | — | — | — | 6/6 ✅ | — | — | — |
| teapot.3dm | — | — | — | — | 3/3 ✅ | — | — | — |
| disk_brake.3dm | — | — | — | — | 66/66 ✅ | — | — | — |
| extrusions.3dm | — | — | — | — | 7/7 ✅ | — | — | — |
| section_hatch.3dm | — | — | — | — | — | — | — | 3/3 ✅ |
| selective_clipping.3dm | — | — | — | — | — | — | — | 6/6 ✅ |
| my_trimmed_surface.3dm | — | — | — | — | 1/1 ✅ | — | 1/1 ✅ | 1/1 ✅ |
| my_brep.3dm | — | ✅ | ✅ | — | — | — | 1/1 ✅ | — |
| rhino_logo_nurbs.3dm | — | — | — | — | — | — | 6/6 ✅ | — |
