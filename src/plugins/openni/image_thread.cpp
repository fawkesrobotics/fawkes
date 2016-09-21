
/***************************************************************************
 *  image_thread.cpp - OpenNI image provider thread
 *
 *  Created: Thu Mar 17 14:06:39 2011
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

#include "image_thread.h"
#include "utils/setup.h"

#include <core/threading/mutex_locker.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/color/bayer.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/yuvrgb.h>
#include <fvutils/color/rgbyuv.h>

#include <memory>

using namespace fawkes;
using namespace firevision;

/** @class OpenNiImageThread "image_thread.h"
 * OpenNI Image Provider Thread.
 * This thread provides YUV and RGB images from the camera via
 * SharedMemoryImageBuffer to other FireVision plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiImageThread::OpenNiImageThread()
  : Thread("OpenNiImageThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PREPARE)
{
}


/** Destructor. */
OpenNiImageThread::~OpenNiImageThread()
{
}


void
OpenNiImageThread::init()
{
  MutexLocker lock(openni.objmutex_ptr());

  __cfg_copy_mode = CONVERT_YUV;

  __image_gen = new xn::ImageGenerator();
#if __cplusplus >= 201103L
  std::unique_ptr<xn::ImageGenerator> imagegen_uniqueptr(__image_gen);
#else
  std::auto_ptr<xn::ImageGenerator> imagegen_uniqueptr(__image_gen);
#endif

  XnStatus st;

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_IMAGE, __image_gen);

  fawkes::openni::setup_map_generator(*__image_gen, config);

  fawkes::openni::get_usb_info(*__image_gen, __usb_vendor, __usb_product);

  if ( (__usb_vendor == 0x045e) && (__usb_product == 0x02ae) ) {
    // from OpenNI-PrimeSense/XnStreamParams.h:
    // XN_IO_IMAGE_FORMAT_UNCOMPRESSED_BAYER = 6
    // InputFormat should be 6 = uncompressed Bayer for Kinect
    logger->log_debug(name(), "Kinect camera detected, initializing");
    if (__image_gen->SetIntProperty("InputFormat", 6) != XN_STATUS_OK) {
      throw Exception("Failed to set uncompressed bayer input format");
    }
    if (__image_gen->SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT) != XN_STATUS_OK)
    {
      throw Exception("Failed to set pixel format");
    }
    /*
    // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
    // (from ROS openni_camera)
    if (__depth_gen->SetIntProperty ("RegistrationType", 2) != XN_STATUS_OK) {
      throw Exception("Failed to set registration type");
    }
    */
    __cfg_copy_mode = DEBAYER_BILINEAR;
    try {
      std::string debayering = config->get_string("/plugins/openni-image/debayering");
      if (debayering == "bilinear") {
        __cfg_copy_mode = DEBAYER_BILINEAR;
      } else if (debayering == "nearest_neighbor") {
        __cfg_copy_mode = DEBAYER_NEAREST_NEIGHBOR;
      } else {
        logger->log_warn(name(), "Unknown de-bayering mode '%s', using bilinear instead.",
			 debayering.c_str());
      }
    } catch (Exception &e) {
      logger->log_warn(name(), "No de-bayering mode set, using bilinear.");
    }
  } else {
    logger->log_debug(name(), "PrimeSense camera detected, initializing");
    if (__image_gen->SetIntProperty("InputFormat", 5) != XN_STATUS_OK) {
      throw Exception("Failed to set uncompressed bayer input format");
    }
    if (__image_gen->SetPixelFormat(XN_PIXEL_FORMAT_YUV422) != XN_STATUS_OK) {
      throw Exception("Failed to set pixel format");
    }
    __cfg_copy_mode = CONVERT_YUV;
  }

  __image_md = new xn::ImageMetaData();

  __image_gen->GetMetaData(*__image_md);

  __image_width  = __image_md->XRes();
  __image_height = __image_md->YRes();

  /*
  const char *pixel_format = "unknown";
  switch (__image_gen->GetPixelFormat()) {
  case XN_PIXEL_FORMAT_RGB24:            pixel_format = "RGB24"; __cfg_copy_mode = CONVERT_RGB; break;
  case XN_PIXEL_FORMAT_YUV422:           pixel_format = "YUV422"; break;
  case XN_PIXEL_FORMAT_GRAYSCALE_8_BIT:  pixel_format = "Gray8"; break;
  case XN_PIXEL_FORMAT_GRAYSCALE_16_BIT: pixel_format = "Gray16"; break; 	
  case XN_PIXEL_FORMAT_MJPEG:            pixel_format = "MJPEG"; break; 	
  }

  XnUInt64 input_format;
  if (__image_gen->GetIntProperty("InputFormat", input_format) != XN_STATUS_OK) {
    logger->log_warn(name(), "Failed to get input format");
  }

  logger->log_debug(name(), "Image  format: %s  width: %u  height: %u  input format: %lu",
		    pixel_format, __image_md->XRes(), __image_md->YRes(), input_format);
  */

  __image_buf_yuv =
    new SharedMemoryImageBuffer("openni-image-yuv", YUV422_PLANAR,
                                __image_md->XRes(), __image_md->YRes());

  __image_buf_rgb =
    new SharedMemoryImageBuffer("openni-image-rgb", RGB,
                                __image_md->XRes(), __image_md->YRes());


  __image_gen->StartGenerating();

  __capture_start = new Time(clock);
  __capture_start->stamp_systime();
  // Update once to get timestamp
  __image_gen->WaitAndUpdateData();
  // arbitrarily define the zero reference point,
  // we can't get any closer than this
  *__capture_start -= (long int)__image_gen->GetTimestamp();
  
  imagegen_uniqueptr.release();
}


void
OpenNiImageThread::finalize()
{
  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete __image_gen;
  delete __image_md;
  delete __image_buf_yuv;
  delete __image_buf_rgb;
}


void
OpenNiImageThread::loop()
{
  MutexLocker lock(openni.objmutex_ptr());
  bool is_image_new = __image_gen->IsDataNew();
  __image_gen->GetMetaData(*__image_md);
  const XnUInt8 * const      image_data = __image_md->Data();
  fawkes::Time ts = *__capture_start + (long int)__image_gen->GetTimestamp();
  lock.unlock();

  if (is_image_new && (__image_buf_yuv->num_attached() > 1)) {
    __image_buf_yuv->lock_for_write();
    if (__cfg_copy_mode == DEBAYER_BILINEAR) {
      bayerGRBG_to_yuv422planar_bilinear(image_data, __image_buf_yuv->buffer(),
					 __image_width, __image_height);
    } else if (__cfg_copy_mode == CONVERT_YUV) {
      yuv422packed_to_yuv422planar(image_data, __image_buf_yuv->buffer(),
				   __image_width, __image_height);
    } else if (__cfg_copy_mode == CONVERT_RGB) {
      rgb_to_yuv422planar_plainc(image_data, __image_buf_yuv->buffer(),
				 __image_width, __image_height);
    } else if (__cfg_copy_mode == DEBAYER_NEAREST_NEIGHBOR) {
      bayerGRBG_to_yuv422planar_nearest_neighbour(image_data,
						  __image_buf_yuv->buffer(),
						  __image_width, __image_height);
    }
    __image_buf_yuv->set_capture_time(&ts);
    __image_buf_yuv->unlock();
  }

  if (is_image_new && (__image_buf_rgb->num_attached() > 1)) {
    __image_buf_rgb->lock_for_write();
    if (__cfg_copy_mode == DEBAYER_BILINEAR) {
      bayerGRBG_to_rgb_bilinear(image_data, __image_buf_rgb->buffer(),
                                __image_width, __image_height);
    } else if (__cfg_copy_mode == CONVERT_YUV) {
      yuv422packed_to_rgb_plainc(image_data, __image_buf_rgb->buffer(),
                                 __image_width, __image_height);
    } else if (__cfg_copy_mode == CONVERT_RGB) {
      memcpy(__image_buf_rgb->buffer(), image_data,
	     colorspace_buffer_size(RGB, __image_width, __image_height));
    } else if (__cfg_copy_mode == DEBAYER_NEAREST_NEIGHBOR) {
      bayerGRBG_to_rgb_nearest_neighbour(image_data, __image_buf_rgb->buffer(),
                                         __image_width, __image_height);
    }
    __image_buf_rgb->set_capture_time(&ts);
    __image_buf_rgb->unlock();
  }
}
