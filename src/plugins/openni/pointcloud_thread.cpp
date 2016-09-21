
/***************************************************************************
 *  pointcloud_thread.cpp - OpenNI point cloud provider thread
 *
 *  Created: Fri Mar 25 23:49:11 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "pointcloud_thread.h"
#include "image_thread.h"
#include "utils/setup.h"

#include <core/threading/mutex_locker.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/base/types.h>
#include <fvutils/color/rgb.h>
#ifdef HAVE_PCL
#  include <pcl_utils/utils.h>
#endif

#include <memory>

using namespace fawkes;
using namespace firevision;

/** @class OpenNiPointCloudThread "pointcloud_thread.h"
 * OpenNI Point Cloud Provider Thread.
 * This thread provides a point cloud calculated from the depth image
 * acquired via OpenNI and provides it as a
 * SharedMemoryImageBuffer to other FireVision plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param img_thread OpenNI image thread, used for XYZRGB point clouds
 */
OpenNiPointCloudThread::OpenNiPointCloudThread(OpenNiImageThread *img_thread)
  : Thread("OpenNiPointCloudThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE)
{
  __img_thread = img_thread;
}


/** Destructor. */
OpenNiPointCloudThread::~OpenNiPointCloudThread()
{
}


void
OpenNiPointCloudThread::init()
{
  MutexLocker lock(openni.objmutex_ptr());

  __image_rgb_buf = NULL;

  __depth_gen = new xn::DepthGenerator();
#if __cplusplus >= 201103L
  std::unique_ptr<xn::DepthGenerator> depthgen_uniqueptr(__depth_gen);
  std::unique_ptr<xn::ImageGenerator> imagegen_uniqueptr(__image_gen);
#else
  std::auto_ptr<xn::DepthGenerator> depthgen_uniqueptr(__depth_gen);
  std::auto_ptr<xn::ImageGenerator> imagegen_uniqueptr(__image_gen);
#endif

  __image_gen = new xn::ImageGenerator();

  XnStatus st;

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, __depth_gen);
  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_IMAGE, __image_gen);
  fawkes::openni::setup_map_generator(*__image_gen, config);
  fawkes::openni::setup_map_generator(*__depth_gen, config);

  __depth_md = new xn::DepthMetaData();
  __depth_gen->GetMetaData(*__depth_md);

  __cfg_register_depth_image = false;
  try {
    __cfg_register_depth_image = config->get_bool("/plugins/openni/register_depth_image");
  } catch (Exception &e) {}

  __cfg_frame_depth = config->get_string("/plugins/openni/frame/depth");
  __cfg_frame_image = config->get_string("/plugins/openni/frame/image");

  __pcl_xyz_buf = new SharedMemoryImageBuffer("openni-pointcloud-xyz",
					  CARTESIAN_3D_FLOAT,
					  __depth_md->XRes(), __depth_md->YRes());

  __pcl_xyz_buf->set_frame_id(__cfg_register_depth_image ? __cfg_frame_image.c_str() : __cfg_frame_depth.c_str());


  __pcl_xyzrgb_buf = new SharedMemoryImageBuffer("openni-pointcloud-xyzrgb",
                                                 CARTESIAN_3D_FLOAT_RGB,
                                                 __depth_md->XRes(), __depth_md->YRes());

  __pcl_xyzrgb_buf->set_frame_id(__cfg_register_depth_image ? __cfg_frame_image.c_str() : __cfg_frame_depth.c_str());

  // this is magic from ROS openni_device.cpp, reading code from
  // openni-primesense suggests that SXGA is the base configuration
  XnUInt64 zpd; // zero plane distance
  if ((st = __depth_gen->GetIntProperty("ZPD", zpd)) != XN_STATUS_OK) {
    throw Exception("Failed to get ZPD: %s", xnGetStatusString(st));
  }
  XnDouble pixel_size; // zero plane pixel size
  if ((st = __depth_gen->GetRealProperty("ZPPS", pixel_size)) != XN_STATUS_OK) {
    throw Exception("Failed to get ZPPS: %s", xnGetStatusString(st));
  }

  if ((st = __depth_gen->GetIntProperty("NoSampleValue", __no_sample_value))
      != XN_STATUS_OK)
  {
    throw Exception("Failed to get NoSampleValue: %s", xnGetStatusString(st));
  }
  if ((st = __depth_gen->GetIntProperty("ShadowValue", __shadow_value))
      != XN_STATUS_OK)
  {
    throw Exception("Failed to get ShadowValue: %s", xnGetStatusString(st));
  }

  __width  = __depth_md->XRes();
  __height = __depth_md->YRes();
  float scale = __width / (float)XN_SXGA_X_RES;
  if (__cfg_register_depth_image) {
    // magic number taken from ROS/PCL openni_device.cpp
    const float rgb_focal_length_SXGA = 1050;
    __focal_length = rgb_focal_length_SXGA * scale;
  } else {
    __focal_length = ((float)zpd / pixel_size) * scale;
  }
  __foc_const = 0.001 / __focal_length;
  __center_x = (__width  / 2.) - .5f;
  __center_y = (__height / 2.) - .5f;

  __image_gen->StartGenerating();
  __depth_gen->StartGenerating();

  __capture_start = new Time(clock);
  __capture_start->stamp_systime();
  // Update once to get timestamp
  __depth_gen->WaitAndUpdateData();
  // arbitrarily define the zero reference point,
  // we can't get any closer than this
  *__capture_start -= (long int)__depth_gen->GetTimestamp();

  __image_gen->WaitAndUpdateData();

  if (__cfg_register_depth_image) {
    // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
    // (from ROS openni_camera)
    unsigned short usb_vendor = 0, usb_product = 0;
    fawkes::openni::get_usb_info(*__depth_gen, usb_vendor, usb_product);

    if ( (usb_vendor == 0x045e) && (usb_product == 0x02ae) ) {
      if (__depth_gen->SetIntProperty("RegistrationType", 2) != XN_STATUS_OK) {
	throw Exception("Failed to set registration type");
      }
    } else {
      if (__depth_gen->SetIntProperty("RegistrationType", 1) != XN_STATUS_OK) {
	throw Exception("Failed to set registration type");
      }
    }

    logger->log_info(name(), "Setting depth alternate viewpoint to image");
    fawkes::openni::setup_alternate_viewpoint(*__depth_gen, *__image_gen);
  }

  // Fails with "Bad Paramter" on OpenNI 1.3.2.1/PS 5.0.3.3
  //logger->log_info(name(), "Setting depth/image synchronization");
  //fawkes::openni::setup_synchronization(*__depth_gen, *__image_gen);

#ifdef HAVE_PCL
  __cfg_generate_pcl = true;
  try {
    __cfg_generate_pcl = config->get_bool("/plugins/openni-pointcloud/generate-pcl");
  } catch (Exception &e) {}

  if (__cfg_generate_pcl) {
    __pcl_xyz = new pcl::PointCloud<pcl::PointXYZ>();
    __pcl_xyz->is_dense = false;
    __pcl_xyz->width    = __width;
    __pcl_xyz->height   = __height;
    __pcl_xyz->points.resize(__width * __height);
    __pcl_xyz->header.frame_id = __cfg_register_depth_image ? __cfg_frame_image : __cfg_frame_depth;

    __pcl_xyzrgb = new pcl::PointCloud<pcl::PointXYZRGB>();
    __pcl_xyzrgb->is_dense = false;
    __pcl_xyzrgb->width    = __width;
    __pcl_xyzrgb->height   = __height;
    __pcl_xyzrgb->points.resize(__width * __height);
    __pcl_xyzrgb->header.frame_id = __cfg_register_depth_image ? __cfg_frame_image : __cfg_frame_depth;

    pcl_manager->add_pointcloud("openni-pointcloud-xyz", __pcl_xyz);
    pcl_manager->add_pointcloud("openni-pointcloud-xyzrgb", __pcl_xyzrgb);
  }
#endif

  depthgen_uniqueptr.release();
  imagegen_uniqueptr.release();
}


void
OpenNiPointCloudThread::finalize()
{
#ifdef HAVE_PCL
  pcl_manager->remove_pointcloud("openni-pointcloud-xyz");
  pcl_manager->remove_pointcloud("openni-pointcloud-xyzrgb");
#endif

  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete __depth_gen;
  delete __depth_md;
  delete __pcl_xyz_buf;
  delete __pcl_xyzrgb_buf;
  delete __capture_start;
}


void
OpenNiPointCloudThread::fill_xyz_no_pcl(fawkes::Time &ts, const XnDepthPixel * const depth_data)
{
  __pcl_xyz_buf->lock_for_write();
  __pcl_xyz_buf->set_capture_time(&ts);

  pcl_point_t *pclbuf = (pcl_point_t *)__pcl_xyz_buf->buffer();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < __height; ++h) {
    for (unsigned int w = 0; w < __width; ++w, ++idx, ++pclbuf) {
      // Check for invalid measurements
      if (depth_data[idx] == 0 ||
          depth_data[idx] == __no_sample_value ||
          depth_data[idx] == __shadow_value)
      {
        // invalid
        pclbuf->x = pclbuf->y = pclbuf->z = 0.f;
      } else {
        // Fill in XYZ
        pclbuf->x = depth_data[idx] * 0.001f;
        pclbuf->y = -(w - __center_x) * depth_data[idx] * __foc_const;
        pclbuf->z = -(h - __center_y) * depth_data[idx] * __foc_const;
      }
    }
  }

  __pcl_xyz_buf->unlock();
}


void
OpenNiPointCloudThread::fill_xyzrgb_no_pcl(fawkes::Time &ts, const XnDepthPixel * const depth_data)
{
  __pcl_xyzrgb_buf->lock_for_write();
  __pcl_xyzrgb_buf->set_capture_time(&ts);

  pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)__pcl_xyzrgb_buf->buffer();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < __height; ++h) {
    for (unsigned int w = 0; w < __width; ++w, ++idx, ++pclbuf_rgb) {
      // Check for invalid measurements
      if (depth_data[idx] == 0 ||
          depth_data[idx] == __no_sample_value ||
          depth_data[idx] == __shadow_value)
      {
        // invalid
        pclbuf_rgb->x = pclbuf_rgb->y = pclbuf_rgb->z = 0.f;
      } else {
        // Fill in XYZ
        pclbuf_rgb->x = depth_data[idx] * 0.001f;
        pclbuf_rgb->y = -(w - __center_x) * depth_data[idx] * __foc_const;
        pclbuf_rgb->z = -(h - __center_y) * depth_data[idx] * __foc_const;
      }
    }
  }

  fill_rgb_no_pcl();

  __pcl_xyzrgb_buf->unlock();
}


void
OpenNiPointCloudThread::fill_xyz_xyzrgb_no_pcl(fawkes::Time &ts,
                                               const XnDepthPixel * const depth_data)
{
  __pcl_xyz_buf->lock_for_write();
  __pcl_xyz_buf->set_capture_time(&ts);

  __pcl_xyzrgb_buf->lock_for_write();
  __pcl_xyzrgb_buf->set_capture_time(&ts);

  pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)__pcl_xyzrgb_buf->buffer();
  pcl_point_t *pclbuf_xyz = (pcl_point_t *)__pcl_xyz_buf->buffer();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < __height; ++h) {
    for (unsigned int w = 0; w < __width; ++w, ++idx, ++pclbuf_rgb, ++pclbuf_xyz) {
      // Check for invalid measurements
      if (depth_data[idx] == 0 ||
          depth_data[idx] == __no_sample_value ||
          depth_data[idx] == __shadow_value)
      {
        // invalid
        pclbuf_rgb->x = pclbuf_rgb->y = pclbuf_rgb->z = 0.f;
        pclbuf_xyz->x = pclbuf_xyz->y = pclbuf_xyz->z = 0.f;
      } else {
        // Fill in XYZ
        pclbuf_rgb->x = pclbuf_xyz->x = depth_data[idx] * 0.001f;
        pclbuf_rgb->y = pclbuf_xyz->y = -(w - __center_x) * depth_data[idx] * __foc_const;
        pclbuf_rgb->z = pclbuf_xyz->z = -(h - __center_y) * depth_data[idx] * __foc_const;
      }
    }
  }

  fill_rgb_no_pcl();

  __pcl_xyzrgb_buf->unlock();
  __pcl_xyz_buf->unlock();
}


void
OpenNiPointCloudThread::fill_rgb_no_pcl()
{
  if (! __image_rgb_buf) {
    try {
      __image_rgb_buf = new SharedMemoryImageBuffer("openni-image-rgb");
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to open openni-image-rgb shm image buffer");
      return;
    }
  }

  __img_thread->wait_loop_done();

  pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)__pcl_xyzrgb_buf->buffer();
  RGB_t *imagebuf = (RGB_t *)__image_rgb_buf->buffer();

  for (unsigned int i = 0; i < __width * __height; ++i) {
    pclbuf_rgb->r = imagebuf[i].R;
    pclbuf_rgb->g = imagebuf[i].G;
    pclbuf_rgb->b = imagebuf[i].B;
  }
}



#ifdef HAVE_PCL
void
OpenNiPointCloudThread::fill_xyz(fawkes::Time &ts, const XnDepthPixel * const depth_data)
{
  pcl::PointCloud<pcl::PointXYZ> &pcl = **__pcl_xyz;
  pcl.header.seq += 1;
  pcl_utils::set_time(__pcl_xyz, ts);

  __pcl_xyz_buf->lock_for_write();
  __pcl_xyz_buf->set_capture_time(&ts);

  pcl_point_t *pclbuf = (pcl_point_t *)__pcl_xyz_buf->buffer();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < __height; ++h) {
    for (unsigned int w = 0; w < __width; ++w, ++idx, ++pclbuf) {
      // Check for invalid measurements
      if (depth_data[idx] == 0 ||
          depth_data[idx] == __no_sample_value ||
          depth_data[idx] == __shadow_value)
      {
        // invalid
        pclbuf->x = pclbuf->y = pclbuf->z = 0.f;
        pcl.points[idx].x = pcl.points[idx].y = pcl.points[idx].z = 0.f;
      } else {
        // Fill in XYZ
        pclbuf->x = pcl.points[idx].x = depth_data[idx] * 0.001f;
        pclbuf->y = pcl.points[idx].y = -(w - __center_x) * depth_data[idx] * __foc_const;
        pclbuf->z = pcl.points[idx].z = -(h - __center_y) * depth_data[idx] * __foc_const;
      }
    }
  }

  __pcl_xyz_buf->unlock();
}


void
OpenNiPointCloudThread::fill_xyzrgb(fawkes::Time &ts, const XnDepthPixel * const depth_data)
{
  pcl::PointCloud<pcl::PointXYZRGB> &pcl_rgb = **__pcl_xyzrgb;
  pcl_rgb.header.seq += 1;
  pcl_utils::set_time(__pcl_xyzrgb, ts);

  __pcl_xyzrgb_buf->lock_for_write();
  __pcl_xyzrgb_buf->set_capture_time(&ts);

  pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)__pcl_xyzrgb_buf->buffer();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < __height; ++h) {
    for (unsigned int w = 0; w < __width; ++w, ++idx, ++pclbuf_rgb) {
      // Check for invalid measurements
      if (depth_data[idx] == 0 ||
          depth_data[idx] == __no_sample_value ||
          depth_data[idx] == __shadow_value)
      {
        // invalid
        pclbuf_rgb->x = pclbuf_rgb->y = pclbuf_rgb->z = 0.f;
        pcl_rgb.points[idx].x = pcl_rgb.points[idx].y = pcl_rgb.points[idx].z = 0.f;
      } else {
        // Fill in XYZ
        pclbuf_rgb->x = pcl_rgb.points[idx].x = depth_data[idx] * 0.001f;
        pclbuf_rgb->y = pcl_rgb.points[idx].y = -(w - __center_x) * depth_data[idx] * __foc_const;
        pclbuf_rgb->z = pcl_rgb.points[idx].z = -(h - __center_y) * depth_data[idx] * __foc_const;
      }
    }
  }

  fill_rgb(pcl_rgb);

  __pcl_xyzrgb_buf->unlock();
}


void
OpenNiPointCloudThread::fill_xyz_xyzrgb(fawkes::Time &ts, const XnDepthPixel * const depth_data)
{

  pcl::PointCloud<pcl::PointXYZRGB> &pcl_rgb = **__pcl_xyzrgb;
  pcl_rgb.header.seq += 1;
  pcl_utils::set_time(__pcl_xyzrgb, ts);

  pcl::PointCloud<pcl::PointXYZ> &pcl_xyz = **__pcl_xyz;
  pcl_xyz.header.seq += 1;
  pcl_utils::set_time(__pcl_xyz, ts);

  __pcl_xyz_buf->lock_for_write();
  __pcl_xyz_buf->set_capture_time(&ts);

  __pcl_xyzrgb_buf->lock_for_write();
  __pcl_xyzrgb_buf->set_capture_time(&ts);

  pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)__pcl_xyzrgb_buf->buffer();
  pcl_point_t *pclbuf_xyz = (pcl_point_t *)__pcl_xyz_buf->buffer();

  unsigned int idx = 0;
  for (unsigned int h = 0; h < __height; ++h) {
    for (unsigned int w = 0; w < __width; ++w, ++idx, ++pclbuf_rgb, ++pclbuf_xyz) {
      // Check for invalid measurements
      if (depth_data[idx] == 0 ||
          depth_data[idx] == __no_sample_value ||
          depth_data[idx] == __shadow_value)
      {
        // invalid
        pclbuf_rgb->x = pclbuf_rgb->y = pclbuf_rgb->z = 0.f;
        pcl_rgb.points[idx].x = pcl_rgb.points[idx].y = pcl_rgb.points[idx].z = 0.f;

        pclbuf_xyz->x = pclbuf_xyz->y = pclbuf_xyz->z = 0.f;
        pcl_xyz.points[idx].x = pcl_xyz.points[idx].y = pcl_xyz.points[idx].z = 0.f;
      } else {
        // Fill in XYZ
        pclbuf_rgb->x = pcl_rgb.points[idx].x = pclbuf_xyz->x = pcl_xyz.points[idx].x =
          depth_data[idx] * 0.001f;
        pclbuf_rgb->y = pcl_rgb.points[idx].y = pclbuf_xyz->y = pcl_xyz.points[idx].y =
          -(w - __center_x) * depth_data[idx] * __foc_const;
        pclbuf_rgb->z = pcl_rgb.points[idx].z = pclbuf_xyz->z = pcl_xyz.points[idx].z =
          -(h - __center_y) * depth_data[idx] * __foc_const;
      }
    }
  }

  fill_rgb(pcl_rgb);

  __pcl_xyzrgb_buf->unlock();
  __pcl_xyz_buf->unlock();
}


void
OpenNiPointCloudThread::fill_rgb(pcl::PointCloud<pcl::PointXYZRGB> &pcl_rgb)
{
  if (! __image_rgb_buf) {
    try {
      __image_rgb_buf = new SharedMemoryImageBuffer("openni-image-rgb");
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to open openni-image-rgb shm image buffer");
      return;
    }
  }

  __img_thread->wait_loop_done();

  pcl_point_xyzrgb_t *pclbuf_rgb = (pcl_point_xyzrgb_t *)__pcl_xyzrgb_buf->buffer();
  RGB_t *imagebuf = (RGB_t *)__image_rgb_buf->buffer();

  for (unsigned int i = 0; i < __width * __height; ++i) {
    pclbuf_rgb->r = pcl_rgb.points[i].r = imagebuf[i].R;
    pclbuf_rgb->g = pcl_rgb.points[i].g = imagebuf[i].G;
    pclbuf_rgb->b = pcl_rgb.points[i].b = imagebuf[i].B;
  }
}

#endif


void
OpenNiPointCloudThread::loop()
{
  MutexLocker lock(openni.objmutex_ptr());
  bool is_data_new = __depth_gen->IsDataNew();
  __depth_gen->GetMetaData(*__depth_md);
  const XnDepthPixel * const data = __depth_md->Data();
  // experimental: unlock here as we do not invoke any methods anymore
  // since data has been updated earlier in the sensor hook we should be safe
  lock.unlock();

  bool xyz_requested = (__pcl_xyz_buf->num_attached() > 1)
#ifdef HAVE_PCL
    // 2 is us and the PCL manager of the PointCloudAspect
    || (__cfg_generate_pcl && ((__pcl_xyz.use_count() > 2)))
#endif
    ;
  bool xyzrgb_requested = (__pcl_xyzrgb_buf->num_attached() > 1)
#ifdef HAVE_PCL
    // 2 is us and the PCL manager of the PointCloudAspect
    || (__cfg_generate_pcl && ((__pcl_xyzrgb.use_count() > 2)))
#endif
    ;

  if (is_data_new && (xyz_requested || xyzrgb_requested)) {
    // convert depth to points
    fawkes::Time ts = *__capture_start + (long int)__depth_gen->GetTimestamp();
  
#ifdef HAVE_PCL
    if (__cfg_generate_pcl) {

      if (xyz_requested && xyzrgb_requested) {
        fill_xyz_xyzrgb(ts, data);
      } else if (xyz_requested) {
        fill_xyz(ts, data);
      } else if (xyzrgb_requested) {
        fill_xyzrgb(ts, data);
      }

    } else {
#endif
      if (xyz_requested && xyzrgb_requested) {
        fill_xyz_xyzrgb_no_pcl(ts, data);
      } else if (xyz_requested) {
        fill_xyz_no_pcl(ts, data);
      } else if (xyzrgb_requested) {
        fill_xyzrgb_no_pcl(ts, data);
      }
#ifdef HAVE_PCL
    }
#endif

    // close rgb image buffer if no longer required
    if (! xyzrgb_requested && __image_rgb_buf) {
      delete __image_rgb_buf;
      __image_rgb_buf = NULL;
    }
  }
}
