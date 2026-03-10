# Install script for directory: /home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runner/work/open2open/open2open/build_prev/open2open/opennurbs/android_uuid/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runner/work/open2open/open2open/build_prev/open2open/opennurbs/freetype263/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/runner/work/open2open/open2open/build_prev/open2open/opennurbs/zlib/cmake_install.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/runner/work/open2open/open2open/build_prev/open2open/opennurbs/libopennurbsStatic.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/runner/work/open2open/open2open/build_prev/open2open/opennurbs/CMakeFiles/opennurbsStatic.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opennurbsStatic" TYPE FILE FILES
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_3dm.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_3dm_attributes.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_3dm_properties.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_3dm_settings.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_annotationbase.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_apple_nsfont.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_arc.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_arccurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_archivable_dictionary.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_archive.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_array.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_array_defs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_base32.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_base64.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_beam.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_bezier.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_bitmap.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_bounding_box.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_box.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_brep.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_circle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_color.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_compress.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_compstat.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_cone.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_convex_poly.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_crc.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_curve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_curveonsurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_curveproxy.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_cylinder.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_date.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_decals.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_defines.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_detail.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_dimension.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_dimensionformat.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_dimensionstyle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_dithering.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_ellipse.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_embedded_file.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_error.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_evaluate_nurbs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_extensions.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_file_utilities.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_font.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_fpoint.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_freetype.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_freetype_include.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_fsp.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_fsp_defs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_function_list.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_geometry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_gl.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_ground_plane.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_group.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_hash_table.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_hatch.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_hsort_template.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_instance.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_V2_annotation.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_V5_annotation.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_V5_dimstyle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_defines.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_glyph.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_intersect.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_ipoint.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_knot.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_layer.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_leader.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_light.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_line.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_linear_workflow.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_linecurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_linestyle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_linetype.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_locale.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_lock.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_lookup.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_mapchan.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_material.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_math.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_matrix.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_md5.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_memory.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_mesh.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_mesh_modifiers.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_model_component.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_model_geometry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_nurbscurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_nurbssurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_object.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_object_history.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_objref.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_offsetsurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_optimize.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_parse.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_photogrammetry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_plane.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_planesurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_pluginlist.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_point.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_pointcloud.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_pointgeometry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_pointgrid.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_polycurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_polyedgecurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_polyline.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_polylinecurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_post_effects.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_private_wrap.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_private_wrap_defs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_progress_reporter.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_qsort_template.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_quacksort_template.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_quaternion.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_rand.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_render_channels.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_render_content.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_rendering.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_revsurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_rtree.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_safe_frame.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sectionstyle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sha1.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_skylight.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sleeplock.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sphere.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_std_string.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_string.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_string_value.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_subd.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_subd_data.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sumsurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sun.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_surface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_surfaceproxy.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_symmetry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_system.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_system_compiler.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_system_runtime.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_terminator.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_testclass.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_text.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_text_style.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textcontext.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textdraw.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textglyph.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textiterator.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textlog.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textobject.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textrun.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_texture.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_texture_mapping.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_topology.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_torus.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_unicode.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_userdata.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_uuid.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_version.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_version_number.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_viewport.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_wip.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_workspace.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_xform.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_xml.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_zlib.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenNURBS.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenNURBS.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenNURBS.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/runner/work/open2open/open2open/build_prev/open2open/opennurbs/libOpenNURBS.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenNURBS.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenNURBS.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libOpenNURBS.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/runner/work/open2open/open2open/build_prev/open2open/opennurbs/CMakeFiles/OpenNURBS.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/OpenNURBS" TYPE FILE FILES
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_3dm.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_3dm_attributes.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_3dm_properties.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_3dm_settings.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_annotationbase.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_apple_nsfont.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_arc.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_arccurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_archivable_dictionary.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_archive.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_array.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_array_defs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_base32.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_base64.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_beam.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_bezier.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_bitmap.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_bounding_box.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_box.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_brep.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_circle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_color.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_compress.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_compstat.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_cone.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_convex_poly.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_crc.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_curve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_curveonsurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_curveproxy.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_cylinder.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_date.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_decals.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_defines.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_detail.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_dimension.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_dimensionformat.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_dimensionstyle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_dithering.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_ellipse.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_embedded_file.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_error.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_evaluate_nurbs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_extensions.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_file_utilities.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_font.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_fpoint.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_freetype.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_freetype_include.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_fsp.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_fsp_defs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_function_list.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_geometry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_gl.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_ground_plane.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_group.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_hash_table.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_hatch.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_hsort_template.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_instance.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_V2_annotation.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_V5_annotation.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_V5_dimstyle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_defines.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_internal_glyph.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_intersect.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_ipoint.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_knot.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_layer.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_leader.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_light.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_line.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_linear_workflow.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_linecurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_linestyle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_linetype.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_locale.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_lock.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_lookup.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_mapchan.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_material.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_math.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_matrix.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_md5.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_memory.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_mesh.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_mesh_modifiers.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_model_component.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_model_geometry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_nurbscurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_nurbssurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_object.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_object_history.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_objref.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_offsetsurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_optimize.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_parse.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_photogrammetry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_plane.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_planesurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_pluginlist.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_point.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_pointcloud.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_pointgeometry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_pointgrid.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_polycurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_polyedgecurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_polyline.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_polylinecurve.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_post_effects.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_private_wrap.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_private_wrap_defs.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_progress_reporter.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_qsort_template.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_quacksort_template.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_quaternion.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_rand.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_render_channels.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_render_content.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_rendering.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_revsurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_rtree.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_safe_frame.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sectionstyle.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sha1.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_skylight.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sleeplock.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sphere.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_std_string.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_string.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_string_value.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_subd.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_subd_data.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sumsurface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_sun.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_surface.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_surfaceproxy.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_symmetry.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_system.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_system_compiler.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_system_runtime.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_terminator.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_testclass.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_text.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_text_style.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textcontext.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textdraw.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textglyph.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textiterator.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textlog.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textobject.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_textrun.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_texture.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_texture_mapping.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_topology.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_torus.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_unicode.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_userdata.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_uuid.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_version.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_version_number.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_viewport.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_wip.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_workspace.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_xform.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_xml.h"
    "/home/runner/work/open2open/open2open/opennurbs-8.24.25281.15001/opennurbs_zlib.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/runner/work/open2open/open2open/build_prev/open2open/opennurbs/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
