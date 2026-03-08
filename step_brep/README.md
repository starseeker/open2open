# step_brep — Pre-converted STEP shapes in OCCT .brep format

This directory contains OCCT `.brep` files pre-converted from the STEP files
in the `STEP_Models/` submodules.  They are committed here so that
`test_occ_roundtrip` can exercise STEP-derived geometry during debugging
sessions **without** needing the STEP submodules checked out or STEP support
linked into the build.

## Contents

| File | Source STEP file | Notes |
|---|---|---|
| `carry_lever_spring_tool_0.brep` | `STEP_Models/Curta-Type-I-3x/CAD/carry_lever_spring_tool.step` | Single-part spring tool |
| `Curta_Assembly_N.brep` (N=0…19) | `STEP_Models/Curta-Type-I-3x/CAD/Curta Assembly.step` | First 20 shapes (representative sample) |
| `Curta_Assembly_N.brep` (N=34,47,…) | `STEP_Models/Curta-Type-I-3x/CAD/Curta Assembly.step` | Known-failing shapes from test_step_roundtrip |

## Regenerating

To regenerate all shapes (or generate shapes from newly added STEP files),
build the `step2brep` tool and run:

```bash
# After building:
build/open2open/tools/step2brep  STEP_Models/  step_brep/
```

This reads every `.stp`/`.step` file found recursively under `STEP_Models/`
and writes one `.brep` file per extracted solid/shell into `step_brep/`.
Commit the files you want to keep for ongoing regression testing.

## Using in tests

`test_occ_roundtrip step_brep/` runs the full OCCT→ON_Brep→OCCT round-trip on
every `.brep` file here.  CMake also registers `test_occ_roundtrip_step_brep`
as a CTest target that does this automatically.

Expected results (as of last generation): **40/41 pass**; the one known failure
(`carry_lever_spring_tool_0.brep`) hits a seam-trim ISO-classification issue
that is tracked separately.
