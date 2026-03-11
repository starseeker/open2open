// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// fcstd_convert.h — FreeCAD .FCStd archive reader.
//
// Provides read-only access to the metadata and display colours stored in a
// FreeCAD `.FCStd` archive, complementing the B-Rep conversion functions in
// `brep_convert.h`.
//
// A `.FCStd` file is a standard ZIP archive containing:
//
//   Document.xml      — object tree (type, name, label, shape file mapping)
//   GuiDocument.xml   — per-object display settings (colour, visibility …)
//   PartShape.brp,    — OpenCASCADE B-Rep shapes in OCCT BRep text format
//   PartShape1.brp …
//   DiffuseColor,     — per-face colour lists (binary, one per shape)
//   DiffuseColor1 …
//
// DiffuseColor binary format
// --------------------------
// Each `DiffuseColor` file contains:
//   uint32  count             (little-endian: number of faces)
//   uint32  color[count]      (per face, packed as 0xRRGGBBAA little-endian)
//
// Note: FreeCAD stores alpha = 0 for fully opaque faces (inverted convention).
//
// Usage
// -----
//   open2open::FcstdDoc doc;
//   if (open2open::ReadFcstdDoc("/path/to/Model.FCStd", doc)) {
//       for (auto& obj : doc.objects) {
//           std::printf("name=%s brp=%s faces=%zu\n",
//                       obj.label.c_str(), obj.brp_file.c_str(),
//                       obj.face_colors.size());
//       }
//   }
//
// Dependencies
// ------------
// libzip (compile-time) — link with -lzip.  When the library is built without
// libzip the functions are not compiled; callers can guard with
// OPEN2OPEN_HAVE_LIBZIP.

#ifndef OPEN2OPEN_FCSTD_CONVERT_H
#define OPEN2OPEN_FCSTD_CONVERT_H

#include <cstdint>
#include <string>
#include <vector>

namespace open2open {

// ---------------------------------------------------------------------------
// Per-object display colour  (packed 0xRRGGBBAA, alpha=0 → fully opaque)
// ---------------------------------------------------------------------------
using FcstdColor = std::uint32_t;

// Convenience helpers for unpacking a FcstdColor.
inline std::uint8_t FcstdColorR(FcstdColor c) { return (c >> 24) & 0xFFu; }
inline std::uint8_t FcstdColorG(FcstdColor c) { return (c >> 16) & 0xFFu; }
inline std::uint8_t FcstdColorB(FcstdColor c) { return (c >>  8) & 0xFFu; }
// alpha=0 means fully opaque in FreeCAD's convention.
inline std::uint8_t FcstdColorA(FcstdColor c) { return (c      ) & 0xFFu; }

// Convert FreeCAD packed color to a normalised [0,1] RGBA tuple.
// FreeCAD alpha=0 → standard alpha=1 (opaque).
inline void FcstdColorToRGBAf(FcstdColor c,
                               float& r, float& g, float& b, float& a)
{
    r = FcstdColorR(c) / 255.0f;
    g = FcstdColorG(c) / 255.0f;
    b = FcstdColorB(c) / 255.0f;
    std::uint8_t fc_alpha = FcstdColorA(c);
    a = (fc_alpha == 0) ? 1.0f : (1.0f - fc_alpha / 255.0f);
}

// ---------------------------------------------------------------------------
// Parse a DiffuseColor binary blob (in memory) into per-face colors.
//
// @param data       Pointer to the raw bytes (e.g. content of DiffuseColor).
// @param size       Number of bytes.
// @param colors_out Receives count packed FcstdColor values; cleared first.
// @return           true on success; false if the data is truncated/invalid.
// ---------------------------------------------------------------------------
bool ParseDiffuseColors(const void*              data,
                        std::size_t              size,
                        std::vector<FcstdColor>& colors_out);

// ---------------------------------------------------------------------------
// Information about one geometric object inside an FCStd archive.
// ---------------------------------------------------------------------------
struct FcstdObject {
    std::string name;        ///< Internal name used as key (e.g. "Part__Feature")
    std::string type;        ///< FreeCAD object type (e.g. "Part::Feature")
    std::string label;       ///< User-facing label (e.g. "ESQ-126-38-G-D_socket")
    std::string brp_file;    ///< Path inside ZIP to the BRep file (e.g. "PartShape.brp")
    FcstdColor  shape_color  = 0xC8C8C8FFu; ///< Overall shape colour (packed RGBA)
    bool        visible      = true;
    std::vector<FcstdColor> face_colors; ///< Per-face colours (may be empty)
};

// ---------------------------------------------------------------------------
// Document-level metadata extracted from Document.xml.
// ---------------------------------------------------------------------------
struct FcstdDocMeta {
    std::string label;        ///< Document label (e.g. "ESQ-126-38-G-D")
    std::string created_by;
    std::string creation_date;
    std::string last_modified_date;
    std::string comment;
    std::string company;
};

// ---------------------------------------------------------------------------
// Everything read from one FCStd archive.
// ---------------------------------------------------------------------------
struct FcstdDoc {
    FcstdDocMeta            meta;
    std::vector<FcstdObject> objects; ///< Only objects with a BRep shape file
};

// ---------------------------------------------------------------------------
// Read an FCStd archive and populate a FcstdDoc.
//
// Reads Document.xml, GuiDocument.xml, and all DiffuseColor binary blobs.
// Objects that have no associated BRep file (e.g. App::Origin, App::Line) are
// silently skipped.
//
// @param path  Filesystem path to the .FCStd file.
// @param doc   Output: populated on success; unchanged on failure.
// @return      true if the archive was opened and at least Document.xml was
//              parsed; false if the file could not be opened as a ZIP archive.
// ---------------------------------------------------------------------------
bool ReadFcstdDoc(const std::string& path, FcstdDoc& doc);

} // namespace open2open

#endif // OPEN2OPEN_FCSTD_CONVERT_H
