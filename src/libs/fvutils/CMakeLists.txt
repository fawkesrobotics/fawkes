set(fvutils_SOURCES)
list(
  APPEND
  fvutils_SOURCES
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
  list(APPEND fvutils_SOURCES adapters/pcl.cpp)
else()
  message(WARNING "Omitting PCL adapter (pcl[-devel] not installed)")
endif()
if(LIBJPEG_FOUND)
  list(APPEND fvutils_SOURCES compression/jpeg_compressor_libjpeg.cpp
       compression/jpeg_decompressor.cpp writers/jpeg.cpp readers/jpeg.cpp)
else()
  message(WARNING "Omitting JPEG support (No libjpeg[-devel] installed)")
endif()
if(LIBPNG_FOUND)
  list(APPEND fvutils_SOURCES readers/png.cpp writers/writer.cpp)
else()
  message(WARNING "Omitting PNG support (No libpng[-devel] installed)")
endif()

if(OPENCV_FOUND)
  list(APPEND fvutils_SOURCES adapters/cvmatadapter.cpp # something about
                                                        # opencv2?!
  )
else()
  message(
    WARNING
      "Omitting OpenCV dependent stuff in the fvutils (No opencv[-devel] installed)"
  )
endif()

add_library(fvutils SHARED ${fvutils_SOURCES})
depend_on_fvconf(fvutils)
target_compile_options(
  fvutils PUBLIC -Wno-unknown-pragmas -Wno-deprecated-declarations
                 -Wno-oerloaded-virtual)
target_link_libraries(fvutils fawkescore fawkesutils fawkesnetcomm
                      fawkeslogging)

if(PCL_FOUND)
  depend_on_pcl(fvutils)
  target_link_libraries(fvutils fawkespcl_utils)
endif()
