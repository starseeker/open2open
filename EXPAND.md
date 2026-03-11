# Expanding open2open: Round-Trip Capabilities Beyond B-Rep

## Executive Summary

The open2open library now achieves reliable NURBS B-Rep round-trips between
openNURBS (Rhino/BRL-CAD) and OpenCASCADE (OCCT/FreeCAD).  The next step is
to identify and implement additional data types that can be faithfully
translated, building toward a high-fidelity conversion pipeline between
Rhino's `.3dm` format and FreeCAD's `.FCStd` format.

This document catalogues:

1. All data categories present in each system.
2. Round-trip feasibility, data loss, and fidelity notes for each category.
3. A ranked work-estimate table.
4. A phased implementation roadmap with the B-Rep core as the foundation.

---

## 1. Data Inventory

### 1.1 openNURBS / Rhino `.3dm`

The openNURBS SDK exposes the following categories through the `ONX_Model`
container:

| Category | Key class(es) | Notes |
|---|---|---|
| **NURBS geometry** | `ON_NurbsCurve`, `ON_NurbsSurface` | Full rational B-spline support |
| **B-Rep solids** | `ON_Brep` | Faces, edges, loops, trims, vertices |
| **Polygon meshes** | `ON_Mesh` | Triangles & quads; optional normals, UV, vertex colors |
| **Point clouds** | `ON_PointCloud` | Structured or unstructured; optional normals, colors |
| **SubD surfaces** | `ON_SubD` | Catmull-Clark limit surfaces |
| **Annotation** | `ON_Annotation`, `ON_Dimension`, `ON_Leader`, `ON_Text` | GD&T / drafting |
| **Hatch fills** | `ON_Hatch` | 2-D hatch patterns |
| **Lights** | `ON_Light` | Directional, point, spot, ambient |
| **Instance defs** | `ON_InstanceDefinition` | Reusable block library |
| **Instance refs** | `ON_InstanceRef` | Placed block with transform |
| **Layers** | `ON_Layer` | Hierarchical; per-layer color, linetype, visibility |
| **Materials** | `ON_Material` | Diffuse / specular / emissive; transparency; textures |
| **Object attributes** | `ON_3dmObjectAttributes` | Per-object name, layer, material, display color, URL |
| **Rendering info** | `ON_RenderContent` | Rendering-engine specific plug-in data |
| **Embedded files** | `ON_EmbeddedFile` | Arbitrary blobs stored in the 3dm archive |
| **Document settings** | `ON_3dmSettings` | Units, tolerance, active viewport |
| **Views / cameras** | `ON_3dmView` | Named views with camera + projection |
| **Groups** | `ON_Group` | Flat named grouping of objects |
| **Text styles** | `ON_TextStyle` | Font / dimension formatting |
| **Line patterns** | `ON_Linetype` | Dash patterns |

### 1.2 OpenCASCADE / FreeCAD `.FCStd`

OCCT's native data model and the XCAF document framework provide:

| Category | Key class(es) / API | Notes |
|---|---|---|
| **NURBS geometry** | `Geom_BSplineCurve`, `Geom_BSplineSurface` | Full rational support |
| **B-Rep topology** | `TopoDS_Shape`, `BRep_Builder` | Faces, edges, loops, wires, vertices, solids, compounds |
| **Polygon meshes** | `Poly_Triangulation` | Triangles only; optional UV, normals |
| **Point sets** | `TColgp_Array1OfPnt` + custom | No first-class point-cloud object |
| **Visualization mesh** | `StdPrs_*`, `BRepMesh_*` | Display tessellations attached to B-Rep faces |
| **Assemblies** | `XDE / TDocStd_Document`, `XCAFDoc_ShapeTool` | Assembly tree with product / part hierarchy |
| **Colors** | `XCAFDoc_Color`, `XCAFDoc_ColorTool` | Per-shape/layer color with transparency |
| **Names / labels** | `XCAFDoc_Name`, `TDF_Label` | Per-node string name |
| **Locations** | `XCAFDoc_Location`, `TopLoc_Location` | Per-instance rigid-body placement |
| **Layers** | `XCAFDoc_LayerTool` | Flat layer-to-shape mapping |
| **Materials** | `XCAFDoc_MaterialTool` | Name, density; limited rendering properties |
| **PMI / GD&T** | `XCAFDoc_DimTol`, `XCAFPrs_*` | Dimensioning, tolerances (GD&T) |
| **STEP metadata** | `STEPCAFControl_Reader/Writer` | Authors, organisations, descriptions |
| **Views** | `XCAFDoc_ViewTool` | Named camera definitions |
| **Clipping planes** | `XCAFDoc_ClippingPlaneTool` | Named clipping planes |
| **Notes / PMI** | `XCAFNoteObjects_*` | Textual and balloon annotations |
| **Document settings** | `TPrsStd_*`, `BinXCAF_*` | Units and tolerances stored in TDF tree |

**FreeCAD `.FCStd` specifics:**  A `.FCStd` file is a ZIP archive containing:

| File | Content |
|---|---|
| `Document.xml` | Object tree (type, name, parent, properties) in XML |
| `GuiDocument.xml` | Viewport state, display modes, per-object display color |
| `PartShape.brp`, `PartShape1.brp`, … | OCCT B-Rep shapes (BRep text format) |
| `DiffuseColor`, `DiffuseColor1`, … | Per-face diffuse color (binary float RGBA array) |
| `*.mesh` | FEM mesh files (FreeCAD FemMesh format) |
| Thumbnails, embedded images | PNG previews |

---

## 2. Round-Trip Analysis

### Legend
- ✅ Supported now (implemented in this library)
- 🟨 Partial support / data loss
- 🔲 Not yet implemented — feasible
- ❌ Not feasible or not meaningful to convert

---

### 2.1 Geometry

| Feature | ON→OCCT | OCCT→ON | Fidelity | Notes |
|---|---|---|---|---|
| NURBS curves (3-D) | ✅ | ✅ | Exact | Knot vectors, weights, periodic flag all preserved |
| NURBS surfaces | ✅ | ✅ | Exact | Knot structure, rational weights preserved |
| B-Rep topology | ✅ | ✅ | High | 99.6% pass on 1 200+ test models |
| Polygon meshes | ✅ | ✅ | High | Implemented in `mesh_convert.h`; vertex colors dropped |
| NURBS curves (2-D, trim) | ✅ | ✅ | Exact | Used internally for B-Rep |
| Analytic surfaces (plane, cyl, cone, sphere, torus) | ✅ | ✅ | High | Converted to NURBS; sphere singularity handled |
| Point clouds | 🔲 | 🔲 | Medium | openNURBS `ON_PointCloud` ↔ `TColgp_Array1OfPnt`; OCCT has no standard container |
| SubD surfaces | 🔲 | ❌ | Low | openNURBS `ON_SubD` has no OCCT equivalent; tessellate to mesh as fallback |
| Extrusions | 🟨 | 🟨 | Medium | openNURBS `ON_Extrusion` converted to B-Rep via `BrepForm()`; OCCT has no ON_Extrusion |

### 2.2 Colour and Material

| Feature | ON→OCCT | OCCT→ON | Fidelity | Notes |
|---|---|---|---|---|
| Object display colour | ✅ | ✅ | High | `ON_3dmObjectAttributes::m_color` ↔ `XCAFDoc_Color` RGBA |
| Layer colour | ✅ | ✅ | High | `ON_Layer::Color()` ↔ `XCAFDoc_Color` on layer label |
| Diffuse / specular material | ✅ | ✅ | Medium | `ON_Material` fields ↔ `XCAFDoc_MaterialTool`; OCCT loses shine/emissive |
| Transparency | ✅ | ✅ | High | Both systems store alpha 0–1 |
| Texture maps | 🔲 | ❌ | Low | openNURBS embeds texture paths; OCCT/XCAF has no standard texture mapping |
| Physically-based materials | ❌ | ❌ | None | Rhino PBR data is proprietary; FreeCAD uses Arch material properties |
| Vertex colors (mesh) | 🟨 | ❌ | None | `ON_Mesh::m_C[]` has no `Poly_Triangulation` equivalent |
| Per-face FreeCAD colours | ✅ | 🟨 | Medium | FreeCAD `DiffuseColor` binary files ↔ per-face XCAF colour |
| Render content (Rhino plug-in) | ❌ | ❌ | None | Proprietary rendering engine data |

### 2.3 Hierarchy and Organisation

| Feature | ON→OCCT | OCCT→ON | Fidelity | Notes |
|---|---|---|---|---|
| Object name | ✅ | ✅ | Exact | `ON_3dmObjectAttributes::m_name` ↔ `XCAFDoc_Name` / TDF label |
| Layer hierarchy | 🔲 | 🔲 | High | `ON_Layer` parent/child ↔ `XCAFDoc_LayerTool` hierarchy |
| Block instances (xref) | 🔲 | 🔲 | Medium | `ON_InstanceDefinition` + `ON_InstanceRef` ↔ `XCAFDoc_ShapeTool` compounds |
| Assembly structure | 🔲 | 🔲 | Medium | XCAF product tree ↔ ON_Layer + block instances; semantic gap |
| Groups | 🔲 | 🔲 | Medium | `ON_Group` ↔ XCAF compound or layer; groups are flat in openNURBS |
| FreeCAD part design tree | ❌ | ❌ | None | Sketch + feature ops have no openNURBS equivalent |
| FreeCAD `App::Part` containers | 🔲 | 🔲 | Medium | Translate to block instance / layer in 3dm |

### 2.4 Metadata and Annotations

| Feature | ON→OCCT | OCCT→ON | Fidelity | Notes |
|---|---|---|---|---|
| Document title / description | ✅ | ✅ | High | `ON_3dmProperties::m_Notes` ↔ STEP header or TDocStd document name |
| Author / company | ✅ | ✅ | High | `ON_3dmProperties::m_RevisionHistory` ↔ STEP file description |
| UUID | 🔲 | 🔲 | High | `ON_3dmObjectAttributes::m_uuid` ↔ TDF label tag |
| URL references | 🔲 | ❌ | Low | `ON_3dmObjectAttributes::m_url` has no standard OCCT field |
| GD&T / PMI | 🔲 | 🔲 | Medium | `ON_Dimension` ↔ `XCAFDoc_DimTol`; styling may differ |
| Text annotations | 🔲 | 🔲 | Medium | `ON_Text` ↔ `XCAFNoteObjects_Note`; font/style differences |
| Dimension styles | ❌ | ❌ | None | Rhino `ON_DimStyle` has no OCCT counterpart |
| Units / tolerances | ✅ | ✅ | High | `ON_3dmSettings` ↔ XCAF unit and tolerance attributes |

### 2.5 Views and Rendering

| Feature | ON→OCCT | OCCT→ON | Fidelity | Notes |
|---|---|---|---|---|
| Named views / cameras | 🔲 | 🔲 | Medium | `ON_3dmView` ↔ `XCAFDoc_ViewTool`; projection type differences |
| Lights | 🔲 | ❌ | Low | `ON_Light` has no standard XCAF equivalent |
| Clipping planes | 🔲 | 🔲 | Medium | `ON_ClippingPlaneObject` ↔ `XCAFDoc_ClippingPlaneTool` |
| Preview image | 🟨 | 🟨 | Low | Thumbnails embedded differently in each format |

---

## 3. What Cannot Be Preserved — and Why

### 3.1 FreeCAD parametric feature tree

FreeCAD's native files store the full parametric history: Sketch objects,
Pad/Pocket/Fillet/Chamfer operations, Boolean features, FEM meshes, etc.
openNURBS has no concept of parametric history — it only stores evaluated
shapes.  The conversion must evaluate the FreeCAD feature tree to obtain a
final B-Rep or mesh and discard the construction history.

### 3.2 Rhino SubD surfaces

openNURBS `ON_SubD` stores a Catmull-Clark control cage.  OCCT 7.9 has no
SubD surface class.  Conversion must tessellate the SubD limit surface to a
polygon mesh, losing the smooth limit-surface definition.

### 3.3 Rhino Render Content (plug-in materials)

Rhino embeds rendering-engine specific XML blobs in the 3dm file for each
plug-in (V-Ray, Enscape, Cycles, etc.).  These are opaque to openNURBS and
cannot be mapped to OCCT.

### 3.4 Mesh vertex colors

`ON_Mesh::m_C[]` stores per-vertex RGBA colors.  OCCT's `Poly_Triangulation`
has no per-vertex color field.  Colors can only be preserved at the
per-object level via XCAF, not per-vertex.

### 3.5 Rhino text / dimension styles

`ON_TextStyle` and `ON_DimStyle` define rich typographic and annotation
formatting.  OCCT's annotation framework (`XCAFDoc_DimTol`) focuses on the
geometric meaning of GD&T rather than visual formatting.

### 3.6 FreeCAD FEM meshes

FreeCAD FEM meshes (`*.mesh` files in `.FCStd`) use FreeCAD's internal FEM
format, which can represent higher-order (quadratic) elements not present in
openNURBS.

---

## 4. Work Estimates

Estimated effort for each feature in engineering days, for a developer familiar
with both APIs:

| Priority | Feature | Effort | Dependencies |
|---|---|---|---|
| P0 | ~~B-Rep solids~~ | ~~Done~~ | — |
| P0 | ~~Polygon mesh (tri/quad, normals, UV)~~ | ~~Done~~ | — |
| P1 | ~~**Object name / label**~~ | ~~Done~~ | TDF label API |
| P1 | ~~**Object display colour**~~ | ~~Done~~ | XCAF `XCAFDoc_Color` |
| P1 | ~~**Layer name and colour**~~ | ~~Done~~ | `XCAFDoc_LayerTool`, `ON_Layer` |
| P2 | ~~**Basic material (diffuse, transparency)**~~ | ~~Done~~ | `XCAFDoc_MaterialTool`, `ON_Material` |
| P2 | ~~**FreeCAD per-face colours** (`DiffuseColor` binary)~~ | ~~Done~~ | Custom ZIP + binary parser |
| P2 | ~~**Document metadata** (title, author, date)~~ | ~~Done~~ | `ON_3dmProperties`, TDocStd header |
| P2 | ~~**Units and tolerance**~~ | ~~Done~~ | `ON_3dmSettings`, XCAF units |
| P3 | ~~**Instance definitions / blocks**~~ | ~~Done~~ | `XCAFDoc_ShapeTool`, `ON_InstanceDefinition` |
| P3 | **Assembly tree** | 7–10 d | XCAF product tree → ON_Layer hierarchy |
| P3 | **GD&T / PMI annotations** | 8–12 d | `XCAFDoc_DimTol`, `ON_Dimension` |
| P3 | **Named views / cameras** | 3–5 d | `XCAFDoc_ViewTool`, `ON_3dmView` |
| P4 | **Point clouds** | 3–5 d | `ON_PointCloud` ↔ TColgp arrays |
| P4 | **SubD → mesh fallback** | 2–4 d | `ON_SubD::GetMesh()` tessellation |
| P4 | **Text annotations** | 5–8 d | `XCAFNoteObjects`, `ON_Text` |
| P4 | **Texture maps** | 8–12 d | Embed images in both archives |

---

## 5. FreeCAD ↔ 3dm Conversion Architecture

A complete FreeCAD ↔ Rhino 3dm pipeline requires three layers:

```
┌──────────────────────────────────────────────────────┐
│              FreeCAD (.FCStd) file                   │
│  Document.xml  GuiDocument.xml  PartShape*.brp       │
│  DiffuseColor*  *.mesh  thumbnails                   │
└──────────────────┬───────────────────────────────────┘
                   │ ZIP extract + XML parse
                   ▼
┌──────────────────────────────────────────────────────┐
│          open2open FreeCAD reader/writer             │
│  • XML parse (Document.xml → object tree)            │
│  • BRepTools::Read (*.brp → TopoDS_Shape)            │
│  • DiffuseColor binary parser → face colour map      │
│  • Metadata extraction (name, UUID, label)           │
└──────────────────┬───────────────────────────────────┘
                   │ TopoDS_Shape + metadata
                   ▼
┌──────────────────────────────────────────────────────┐
│          open2open core library (existing)           │
│  • OCCTToON_Brep  (TopoDS → ON_Brep)   ✅           │
│  • ON_BrepToOCCT  (ON_Brep → TopoDS)   ✅           │
│  • OCCTToON_Mesh  (Poly_Tri → ON_Mesh) ✅           │
│  • ON_MeshToOCCT  (ON_Mesh → Poly_Tri) ✅           │
└──────────────────┬───────────────────────────────────┘
                   │ ON_Brep / ON_Mesh + ON_3dmObjectAttributes
                   ▼
┌──────────────────────────────────────────────────────┐
│          open2open openNURBS writer/reader           │
│  • Build ONX_Model (objects + layers + materials)    │
│  • ONX_Model::Write → .3dm file                      │
│  • ONX_Model::Read  → objects + attributes           │
└──────────────────┬───────────────────────────────────┘
                   │ ONX_Model
                   ▼
┌──────────────────────────────────────────────────────┐
│              Rhino (.3dm) file                       │
│  Objects  Layers  Materials  Named views  Groups     │
└──────────────────────────────────────────────────────┘
```

### 5.1 FCStd → 3dm conversion steps

1. **Unzip** `.FCStd` into a temporary directory.
2. **Parse `Document.xml`** to build an object map:
   - Object type, name, label, parent, UUID.
   - Identify which `PartShape*.brp` file each object references.
3. **Parse `GuiDocument.xml`** to extract per-object display colour.
4. **Load DiffuseColor binary blobs** for per-face colour overrides.
5. **Read each `.brp`** with `BRepTools::Read`.
6. **Convert TopoDS → ON_Brep** with `OCCTToON_Brep` (existing).
7. **Build `ON_3dmObjectAttributes`** from parsed metadata:
   - Set `m_name` from Document.xml `Label`.
   - Set `m_uuid` from Document.xml `Uid`.
   - Assign layer index based on FreeCAD `App::Part` container.
   - Assign display colour from GuiDocument.
8. **Build `ONX_Model`**: add geometry objects with attributes, create layers.
9. **Write `.3dm`** with `ONX_Model::Write`.

### 5.2 3dm → FCStd conversion steps

1. **Read `.3dm`** with `ONX_Model::Read`.
2. **Iterate geometry objects** with `ONX_ModelComponentIterator`.
3. For each `ON_Brep`:
   - Convert to `TopoDS_Shape` with `ON_BrepToOCCT`.
   - Write to `PartShape_N.brp` with `BRepTools::Write`.
   - Emit `<Object type="Part::Feature" name="Shape_N"/>` in Document.xml.
4. For each `ON_Mesh`:
   - Convert to `Poly_Triangulation` with `ON_MeshToOCCT`.
   - Encode as FreeCAD mesh format (or embed as OCCT brep if topological).
5. **Build layer → FreeCAD `App::Part` mapping**.
6. **Emit per-object DiffuseColor blobs** from `ON_3dmObjectAttributes`.
7. **Write `Document.xml`** and `GuiDocument.xml`.
8. **Zip all files** into `.FCStd`.

---

## 6. Priority Roadmap

### Phase 1 — Geometry foundation (complete)
- [x] NURBS curves (3-D and 2-D) round-trip
- [x] NURBS surfaces round-trip
- [x] B-Rep topology round-trip (faces, edges, loops, trims, vertices)
- [x] Analytic surface conversion (plane, cylinder, cone, sphere, torus)
- [x] Polygon mesh round-trip (triangles, quads, normals, UV)

### Phase 2 — Basic attributes (P1/P2 items above)
- [x] Object name in `ON_3dmObjectAttributes` ↔ TDF label / XCAF Name
- [x] Object display colour ↔ `XCAFDoc_Color`
- [x] Layer name and colour ↔ `XCAFDoc_LayerTool`
- [x] Basic material (diffuse colour, transparency)
- [x] FreeCAD per-face `DiffuseColor` binary parser (`fcstd_convert` module)
- [x] Document title, author, creation date
- [x] Unit system and linear tolerance

### Phase 3 — Structure (P3 items)
- [x] Block instances / assembly references (`ON_InstanceDefinition` + `ON_InstanceRef` ↔ XCAF assembly)
- [x] Named views / cameras (`ON_3dmView` ↔ `XCAFDoc_ViewTool` + `XCAFView_Object`)
- [ ] Assembly hierarchy (XCAF product tree ↔ layer nesting)
- [ ] GD&T / PMI annotations

### Phase 4 — Extended types (P4 items)
- [ ] Point clouds
- [ ] SubD → mesh fallback
- [ ] Text annotations
- [ ] Embedded texture maps

### Phase 5 — Full FreeCAD ↔ 3dm pipeline
- [ ] `fcstd_reader` module: ZIP + XML → ONX_Model
- [ ] `fcstd_writer` module: ONX_Model → ZIP + XML
- [ ] Per-face `DiffuseColor` round-trip
- [ ] `App::Part` container → ON_Layer hierarchy
- [ ] Integration test: `test_fcstd_3dm_roundtrip`

---

## 7. Key Technical Challenges

### 7.1 Assembly semantics gap

OCCT XCAF uses a strict product/part/instance tree modelled on STEP AP214.
openNURBS uses flat layer lists with optional parent layers and block
instances.  There is no one-to-one mapping: sub-assemblies with multiple
instances of the same component must be either expanded (losing instance
identity) or represented as block instances in 3dm.

**Recommended approach:** Map XCAF product nodes to ON_Layer nodes; map XCAF
shape instances to ON_InstanceRef with transforms.  On the reverse path, map
top-level ON_Layers with children to XCAF product nodes.

### 7.2 FreeCAD GuiDocument colour encoding

FreeCAD's `DiffuseColor` files are raw binary arrays of `float[4]` (RGBA) per
face, in document order.  Parsing these requires reading both `Document.xml`
(to know which shapes exist and their face counts) and `GuiDocument.xml` (to
know which colour file belongs to which shape).

### 7.3 NURBS surface periodicity vs. FreeCAD seam handling

FreeCAD uses OCCT's internal B-Rep, which may produce periodic surfaces with
seam edges that are not present in the original 3dm model.  The current
seam-trim repair code handles most cases but some complex seam
crossings (e.g. speaker grille faces) remain problematic.

### 7.4 ON_Mesh quad faces in OCCT

`Poly_Triangulation` only supports triangles.  When an openNURBS quad mesh
is converted to OCCT it is triangulated (split along the 0-2 diagonal).
On the return path all faces come back as triangles.  If the original quad
topology is needed, an ngon-aware mesh container should be used.

---

## 8. Currently Implemented (`open2open` library)

| Module | Header | Functions |
|---|---|---|
| NURBS geometry | `open2open/geom_convert.h` | `ON_NurbsCurveToOCCT`, `ON_NurbsCurve2dToOCCT`, `ON_NurbsSurfaceToOCCT`, `OCCTCurveToON`, `OCCTSurfaceToON` |
| B-Rep topology | `open2open/brep_convert.h` | `ON_BrepToOCCT`, `OCCTToON_Brep` |
| Polygon mesh | `open2open/mesh_convert.h` | `ON_MeshToOCCT`, `OCCTToON_Mesh` |
| Attributes & metadata | `open2open/attrs_convert.h` | `CreateXCAFDocument`, `ONX_ModelToXCAFDoc`, `XCAFDocToONX_Model` |
| FreeCAD FCStd reader | `open2open/fcstd_convert.h` | `ParseDiffuseColors`, `ReadFcstdDoc` |
