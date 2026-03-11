// Copyright (c) 2026, open2open contributors
// SPDX-License-Identifier: MIT
//
// fcstd_convert.cpp — FreeCAD .FCStd archive reader.
//
// Reads Document.xml, GuiDocument.xml, and DiffuseColor binary blobs from a
// FreeCAD .FCStd ZIP archive and returns structured per-object information
// including per-face colours.
//
// Binary DiffuseColor format (confirmed against FreeCAD App/PropertyColor.cpp):
//   uint32 count                  (little-endian, number of faces)
//   uint32 color[count]           (packed 0xRRGGBBAA, stored little-endian)
// FreeCAD alpha=0 means fully opaque (inverted convention vs. standard).
//
// Dependencies: libzip (link -lzip), OCCT LDOM XML parser (TKernel).

#include "open2open/fcstd_convert.h"

// OCCT LDOM XML parser
#include <LDOMParser.hxx>
#include <LDOM_Document.hxx>
#include <LDOM_Element.hxx>
#include <LDOM_Node.hxx>
#include <LDOM_NodeList.hxx>
#include <LDOMString.hxx>

// libzip
#include <zip.h>

#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace open2open {

// ---------------------------------------------------------------------------
// ParseDiffuseColors: decode a binary DiffuseColor blob.
// Format: uint32 count (LE) + count × uint32 (LE, packed 0xRRGGBBAA).
// ---------------------------------------------------------------------------
bool ParseDiffuseColors(const void*              data,
                        std::size_t              size,
                        std::vector<FcstdColor>& out)
{
    out.clear();
    if (!data || size < 4)
        return false;

    const auto* bytes = static_cast<const unsigned char*>(data);

    // Read count as little-endian uint32
    std::uint32_t count =
        (std::uint32_t)bytes[0]        |
        ((std::uint32_t)bytes[1] << 8) |
        ((std::uint32_t)bytes[2] << 16)|
        ((std::uint32_t)bytes[3] << 24);

    if (count == 0)
        return true; // empty list is valid

    // Each entry is 4 bytes; check we have enough data
    if (size < 4 + (std::size_t)count * 4)
        return false;

    out.resize(count);
    for (std::uint32_t i = 0; i < count; ++i) {
        const auto* p = bytes + 4 + i * 4;
        // Little-endian uint32 → packed 0xRRGGBBAA
        std::uint32_t packed =
            (std::uint32_t)p[0]        |
            ((std::uint32_t)p[1] << 8) |
            ((std::uint32_t)p[2] << 16)|
            ((std::uint32_t)p[3] << 24);
        out[i] = static_cast<FcstdColor>(packed);
    }
    return true;
}

// ---------------------------------------------------------------------------
// RAII wrapper around a libzip archive handle.
// ---------------------------------------------------------------------------
struct ZipGuard {
    zip_t* z = nullptr;
    explicit ZipGuard(zip_t* p) : z(p) {}
    ~ZipGuard() { if (z) { zip_discard(z); z = nullptr; } }
    ZipGuard(const ZipGuard&)            = delete;
    ZipGuard& operator=(const ZipGuard&) = delete;
};

// ---------------------------------------------------------------------------
// Read one file from the archive into a string.  Returns empty on error.
// ---------------------------------------------------------------------------
static std::string ZipReadEntry(zip_t* z, const char* entry_name)
{
    zip_file_t* f = zip_fopen(z, entry_name, 0);
    if (!f)
        return {};

    zip_stat_t st;
    zip_stat_init(&st);
    if (zip_stat(z, entry_name, 0, &st) != 0 ||
        !(st.valid & ZIP_STAT_SIZE))
    {
        zip_fclose(f);
        return {};
    }

    std::string buf;
    buf.resize(st.size);
    zip_int64_t nr = zip_fread(f, &buf[0], st.size);
    zip_fclose(f);
    if (nr < 0 || (std::size_t)nr != st.size)
        return {};
    return buf;
}

// ---------------------------------------------------------------------------
// LDOM helper: safe attribute read (returns empty string if not found).
// ---------------------------------------------------------------------------
static std::string LAttr(const LDOM_Element& elem, const char* name)
{
    LDOMString ls = elem.getAttribute(LDOMString(name));
    if (ls == nullptr)
        return {};
    const char* cs = ls.GetString();
    return cs ? std::string(cs) : std::string();
}

// ---------------------------------------------------------------------------
// Walk an LDOM subtree looking for <Property name="propName"> and return the
// first child element's attribute `childAttr`.
// ---------------------------------------------------------------------------
static std::string FindPropertyValue(const LDOM_Element& parent,
                                     const char* prop_name,
                                     const char* child_tag,
                                     const char* child_attr)
{
    LDOM_NodeList props = parent.getElementsByTagName(LDOMString("Property"));
    int n = props.getLength();
    for (int i = 0; i < n; ++i) {
        LDOM_Node node = props.item(i);
        if (node.getNodeType() != LDOM_Node::ELEMENT_NODE)
            continue;
        LDOM_Element propElem = (const LDOM_Element&)node;
        if (LAttr(propElem, "name") != prop_name)
            continue;
        // Find child element with tag child_tag
        LDOM_NodeList children = propElem.getElementsByTagName(LDOMString(child_tag));
        if (children.getLength() > 0) {
            LDOM_Node cn = children.item(0);
            if (cn.getNodeType() == LDOM_Node::ELEMENT_NODE) {
                LDOM_Element ce = (const LDOM_Element&)cn;
                return LAttr(ce, child_attr);
            }
        }
        break;
    }
    return {};
}

// ---------------------------------------------------------------------------
// Parse Document.xml: extract per-object name, type, label, brp_file.
// Returns a map from object internal name → FcstdObject (partially filled).
// ---------------------------------------------------------------------------
static std::map<std::string, FcstdObject>
ParseDocumentXml(const std::string& xml_content, FcstdDocMeta& meta)
{
    std::map<std::string, FcstdObject> result;
    if (xml_content.empty())
        return result;

    std::istringstream iss(xml_content);
    LDOMParser parser;
    if (parser.parse(iss))
        return result; // parse error

    LDOM_Document doc = parser.getDocument();
    LDOM_Element  root = doc.getDocumentElement();
    if (root == nullptr)
        return result;

    // --- Document-level metadata: look for Properties/Property ---
    {
        meta.label         = FindPropertyValue(root, "Label",            "String",  "value");
        meta.created_by    = FindPropertyValue(root, "CreatedBy",        "String",  "value");
        meta.creation_date = FindPropertyValue(root, "CreationDate",     "String",  "value");
        meta.last_modified_date = FindPropertyValue(root, "LastModifiedDate", "String", "value");
        meta.comment       = FindPropertyValue(root, "Comment",          "String",  "value");
        meta.company       = FindPropertyValue(root, "Company",          "String",  "value");
    }

    // --- Object type list: <Objects><Object type="..." name="..." ...> ---
    {
        LDOM_NodeList objects = root.getElementsByTagName(LDOMString("Object"));
        int n = objects.getLength();
        for (int i = 0; i < n; ++i) {
            LDOM_Node node = objects.item(i);
            if (node.getNodeType() != LDOM_Node::ELEMENT_NODE)
                continue;
            LDOM_Element elem = (const LDOM_Element&)node;
            std::string type = LAttr(elem, "type");
            std::string name = LAttr(elem, "name");
            if (!type.empty() && !name.empty()) {
                // This is the <Object type="Part::Feature" name="..." id="..."/>
                // element in the <Objects> section — record the type.
                result[name].name = name;
                result[name].type = type;
            }
        }
    }

    // --- ObjectData: <Object name="..."><Properties>...</Properties></Object> ---
    // Fills label and brp_file by looking for Property name="Label" and "Shape".
    {
        LDOM_NodeList objects = root.getElementsByTagName(LDOMString("Object"));
        int n = objects.getLength();
        for (int i = 0; i < n; ++i) {
            LDOM_Node node = objects.item(i);
            if (node.getNodeType() != LDOM_Node::ELEMENT_NODE)
                continue;
            LDOM_Element elem = (const LDOM_Element&)node;
            // Only ObjectData/Object elements have Properties children but no type
            if (!LAttr(elem, "type").empty())
                continue;
            std::string name = LAttr(elem, "name");
            if (name.empty())
                continue;

            // Label
            std::string label = FindPropertyValue(elem, "Label", "String", "value");
            if (!label.empty())
                result[name].label = label;

            // Shape → brp file path
            std::string brp = FindPropertyValue(elem, "Shape", "Part", "file");
            if (!brp.empty())
                result[name].brp_file = brp;
        }
    }

    return result;
}

// ---------------------------------------------------------------------------
// Parse GuiDocument.xml: extract per-object ShapeColor and DiffuseColor refs.
// Updates result (FcstdObject::shape_color, diffuse_file field, visibility).
// Returns a map from internal name → diffuse color file name in archive.
// ---------------------------------------------------------------------------
static void ParseGuiDocumentXml(const std::string& xml_content,
                                 std::map<std::string, FcstdObject>& objects,
                                 std::map<std::string, std::string>& diffuse_files)
{
    if (xml_content.empty())
        return;

    std::istringstream iss(xml_content);
    LDOMParser parser;
    if (parser.parse(iss))
        return;

    LDOM_Document doc = parser.getDocument();
    LDOM_Element  root = doc.getDocumentElement();
    if (root == nullptr)
        return;

    // Each <ViewProvider name="..."> holds per-object GUI properties
    LDOM_NodeList vps = root.getElementsByTagName(LDOMString("ViewProvider"));
    int n = vps.getLength();
    for (int i = 0; i < n; ++i) {
        LDOM_Node node = vps.item(i);
        if (node.getNodeType() != LDOM_Node::ELEMENT_NODE)
            continue;
        LDOM_Element vp = (const LDOM_Element&)node;
        std::string name = LAttr(vp, "name");
        if (name.empty())
            continue;

        // ShapeColor: <Property name="ShapeColor"><PropertyColor value="0xRRGGBBAA"/></Property>
        {
            std::string val = FindPropertyValue(vp, "ShapeColor", "PropertyColor", "value");
            if (!val.empty()) {
                try {
                    unsigned long packed = std::stoul(val);
                    objects[name].shape_color = static_cast<FcstdColor>(packed);
                } catch (...) {}
            }
        }

        // Visibility: <Property name="Visibility"><Bool value="true/false"/></Property>
        {
            std::string val = FindPropertyValue(vp, "Visibility", "Bool", "value");
            if (val == "false")
                objects[name].visible = false;
        }

        // DiffuseColor: <Property name="DiffuseColor">
        //                 <ColorList file="DiffuseColor" />
        //               </Property>
        {
            std::string file = FindPropertyValue(vp, "DiffuseColor", "ColorList", "file");
            if (!file.empty())
                diffuse_files[name] = file;
        }
    }
}

// ---------------------------------------------------------------------------
// ReadFcstdDoc: open an FCStd archive and populate a FcstdDoc.
// ---------------------------------------------------------------------------
bool ReadFcstdDoc(const std::string& path, FcstdDoc& doc)
{
    doc = FcstdDoc{};

    int err = 0;
    zip_t* raw_zip = zip_open(path.c_str(), ZIP_RDONLY, &err);
    if (!raw_zip)
        return false;
    ZipGuard guard(raw_zip);

    // Read Document.xml
    std::string doc_xml = ZipReadEntry(raw_zip, "Document.xml");
    if (doc_xml.empty())
        return false; // must have Document.xml

    // Read GuiDocument.xml (optional — may not be present in all FCStd files)
    std::string gui_xml = ZipReadEntry(raw_zip, "GuiDocument.xml");

    // Parse Document.xml → object map + doc meta
    std::map<std::string, FcstdObject> obj_map;
    obj_map = ParseDocumentXml(doc_xml, doc.meta);

    // Parse GuiDocument.xml → fill in shape_color + collect diffuse file refs
    std::map<std::string, std::string> diffuse_file_refs; // obj_name → archive entry
    ParseGuiDocumentXml(gui_xml, obj_map, diffuse_file_refs);

    // For each object with a DiffuseColor reference, read and parse the binary
    for (auto& kv : diffuse_file_refs) {
        const std::string& obj_name  = kv.first;
        const std::string& zip_entry = kv.second;
        if (obj_map.find(obj_name) == obj_map.end())
            continue;

        std::string blob = ZipReadEntry(raw_zip, zip_entry.c_str());
        if (blob.empty())
            continue;

        ParseDiffuseColors(blob.data(), blob.size(),
                           obj_map[obj_name].face_colors);
    }

    // Collect objects that have a BRep file (i.e. have geometric content).
    // Skip origin axes, planes, and other non-geometric objects.
    doc.objects.clear();
    for (auto& kv : obj_map) {
        FcstdObject& obj = kv.second;
        if (obj.brp_file.empty())
            continue;
        doc.objects.push_back(std::move(obj));
    }

    return true;
}

} // namespace open2open
