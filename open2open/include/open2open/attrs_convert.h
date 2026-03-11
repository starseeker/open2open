// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// attrs_convert.h — Attribute and metadata conversion helpers.
//
// These functions translate non-geometry data (object names, display colours,
// layer assignments, materials, document metadata and unit system) between an
// openNURBS ONX_Model and an OpenCASCADE XDE document (TDocStd_Document with
// XCAF attributes).
//
// Availability
// ------------
// This header is only meaningful when the library was compiled with XCAF
// support (TKLCAF + TKXCAF).  The macro OPEN2OPEN_HAVE_XCAF is defined by
// CMake in that case.  When it is absent the header still exists but exposes
// no symbols; callers should guard their code with the same macro.

#ifndef OPEN2OPEN_ATTRS_CONVERT_H
#define OPEN2OPEN_ATTRS_CONVERT_H

#ifdef OPEN2OPEN_HAVE_XCAF

#include <Standard_Handle.hxx>

// Forward declarations — avoids exposing OCCT or openNURBS headers to callers
// that only need the type names.
class TDocStd_Document;
class ONX_Model;

namespace open2open {

/// Create a new, empty XCAF document backed by a minimal in-memory framework.
///
/// The returned document has the DocumentTool (ShapeTool, ColorTool,
/// LayerTool, MaterialTool) already initialised on its main label.
///
/// @return  A new TDocStd_Document handle, never null.
Handle(TDocStd_Document) CreateXCAFDocument();

/// Convert an openNURBS model (ONX_Model) to an XCAF TDocStd_Document.
///
/// All B-Rep geometry objects in the model are converted to TopoDS_Shape and
/// added to the XCAF ShapeTool.  Per-object attributes are propagated as:
///
/// | openNURBS source                         | XCAF destination            |
/// |------------------------------------------|-----------------------------|
/// | `ON_3dmObjectAttributes::m_name`         | `TDataStd_Name` on label    |
/// | `ON_3dmObjectAttributes::m_color`        | `XCAFDoc_ColorTool` (Gen)   |
/// | `ON_3dmObjectAttributes::m_layer_index`  | `XCAFDoc_LayerTool`         |
/// | `ON_3dmObjectAttributes::m_material_index`| `XCAFDoc_MaterialTool`     |
///
/// Layer entries are created from the model's layer table, including the
/// layer colour stored as a `XCAFDoc_ColorTool` entry on the layer label.
///
/// Material entries are created from the model's material table.  The
/// material diffuse colour is stored as a surface colour on the shape label
/// via `XCAFDoc_ColorTool`.  Only the first rendering material with that
/// index is used.
///
/// Document-level metadata:
///   - Model unit system → `XCAFDoc_LengthUnit` scale-factor on doc root
///   - Created-by string → `TDataStd_Name` on doc root
///
/// Instance definitions and references (P3):
///   - Each `ON_InstanceDefinition` → XCAF assembly label with sub-components.
///   - Each `ON_InstanceRef` → XCAF component with `TopLoc_Location` derived
///     from `m_xform`.  Instance refs are collected under a "Scene" assembly.
///
/// Mesh objects (ON_Mesh) in the model are currently skipped.
///
/// @param model  Source openNURBS model.
/// @param tol    Linear tolerance forwarded to ON_BrepToOCCT.
/// @return       A new XCAF document, or a null handle on failure.
Handle(TDocStd_Document) ONX_ModelToXCAFDoc(const ONX_Model& model,
                                            double           tol = 1e-6);

/// Convert an XCAF TDocStd_Document back to an openNURBS ONX_Model.
///
/// Top-level shapes in the XCAF ShapeTool are converted to ON_Brep objects
/// and added to the model.  XCAF attributes are mapped back as:
///
/// | XCAF source                              | openNURBS destination             |
/// |------------------------------------------|-----------------------------------|
/// | `TDataStd_Name` on shape label           | `ON_3dmObjectAttributes::m_name`  |
/// | `XCAFDoc_ColorTool` (Gen/Surf)           | `m_color` + `color_from_object`   |
/// | `XCAFDoc_LayerTool`                      | `ON_Layer` + `m_layer_index`      |
/// | `XCAFDoc_MaterialTool`                   | `ON_Material` + `m_material_index`|
///
/// Layer colours are read from the `XCAFDoc_ColorTool` attribute stored on
/// the layer label.  The model's unit system is initialised from the
/// `XCAFDoc_LengthUnit` scale factor stored on the document root.
///
/// @param doc    Source XCAF document (must not be null).
/// @param model  Destination openNURBS model (reset before writing).
/// @param tol    Linear tolerance forwarded to OCCTToON_Brep.
/// @return       true on success; false if the document is null or conversion
///               fails entirely (individual shape failures are non-fatal).
bool XCAFDocToONX_Model(const Handle(TDocStd_Document)& doc,
                        ONX_Model&                      model,
                        double                          tol = 1e-6);

} // namespace open2open

#endif // OPEN2OPEN_HAVE_XCAF
#endif // OPEN2OPEN_ATTRS_CONVERT_H
