# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

add_library(
  fvutils SHARED
  writers/png.cpp
  writers/compressed.cpp
  writers/fvraw.cpp
  writers/pnm.cpp
  writers/seq_writer.cpp
  statistical/imagediff.cpp
  statistical/histogram.cpp
  statistical/histogram_file.cpp
  statistical/histogram_block.cpp
  colormap/cmfile_block.cpp
  colormap/cmfile_yuvblock.cpp
  colormap/cmfile.cpp
  colormap/colormap.cpp
  colormap/bayes/bayes_generator.cpp
  colormap/bayes/bayes_histos_to_lut.cpp
  colormap/generator.cpp
  colormap/yuvcm.cpp
  net/fuse_imagelist_content.cpp
  net/fuse_server_client_thread.cpp
  net/fuse_transceiver.cpp
  net/fuse_server.cpp
  net/fuse_client.cpp
  net/fuse_client_handler.cpp
  net/fuse_lutlist_content.cpp
  net/fuse_lut_content.cpp
  net/fuse_image_content.cpp
  net/fuse_message_content.cpp
  net/fuse_message.cpp
  base/roi.cpp
  base/vision_master.cpp
  rectification/rectinfo_block.cpp
  rectification/rectfile.cpp
  rectification/rectinfo_lut_block.cpp
  rectification/rectinfo.cpp
  # qa/qa_yuvconv.cpp qa/qa_fuse.cpp qa/qa_jpegbm.cpp qa/qa_camargp.cpp
  # qa/qa_colormap.cpp qa/qa_rectlut.cpp qa/qa_shmlut.cpp qa/qa_createimage.cpp
  # qa/qa_shmimg.cpp
  fileformat/fvfile_block.cpp
  fileformat/fvfile.cpp
  scalers/scaler.cpp
  scalers/lossy.cpp
  draw/field_lines.cpp
  draw/mono_drawer.cpp
  draw/field.cpp
  draw/drawer.cpp
  draw/field_drawer.cpp
  # compression/jpeg_compressor_mmal.cpp # no RASPI stuff available
  compression/imagecompressor.cpp
  compression/imagedecompressor.cpp
  compression/jpeg_compressor.cpp
  color/yuvrgb.cpp
  color/zauberstab.cpp
  color/rgb.cpp
  color/conversions.cpp
  color/yuv.cpp
  color/colorspaces.cpp
  color/color_object_map.cpp
  color/rgbyuv.cpp
  color/bayer.cpp
  camera/tracker.cpp
  ipc/shm_image.cpp
  ipc/shm_lut.cpp
  readers/reader.cpp
  readers/fvraw.cpp
  readers/pnm.cpp
  system/camargp.cpp)
if(PCL_FOUND)
  target_sources(fvutils PRIVATE adapters/pcl.cpp)
else()
  build_skipped_message("fvutils PCL adapter" "pcl[-devel]")
endif()
optional_depend_on_pkgconfig_libs(fvutils libjpeg libjpeg_dep_found)
if(libjpeg_dep_found)
  target_sources(
    fvutils
    PRIVATE compression/jpeg_compressor_libjpeg.cpp
            compression/jpeg_decompressor.cpp writers/jpeg.cpp readers/jpeg.cpp)
else()
  build_skipped_message("fvutils JPEG support" "libjpeg[-devel]")
endif()
optional_depend_on_pkgconfig_libs(fvutils libpng libpng_dep_found)
if(libpng_dep_found)
  target_sources(fvutils PRIVATE readers/png.cpp writers/writer.cpp)
else()
  build_skipped_message("fvutils PNG support" "libpng[-devel]")
endif()

optional_depend_on_pkgconfig_libs(fvutils opencv opencv_dep_found)
if(opencv_dep_found)
  target_sources(fvutils PRIVATE adapters/cvmatadapter.cpp)
else()
  build_skipped_message("fvutils OpenCV support" "opencv[-devel]")
endif()

depend_on_fvconf(fvutils)
target_compile_options(
  fvutils PUBLIC -Wno-unknown-pragmas -Wno-deprecated-declarations
                 -Wno-overloaded-virtual)
target_link_libraries(fvutils fawkescore fawkesutils fawkesnetcomm
                      fawkeslogging)

if(PCL_FOUND)
  depend_on_pcl(fvutils)
  build_depends_on(fvutils fawkespcl_utils)
  target_link_libraries(fvutils fawkespcl_utils)
endif()
