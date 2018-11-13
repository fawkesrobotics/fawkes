
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

  cfg_copy_mode_ = CONVERT_YUV;

  image_gen_ = new xn::ImageGenerator();
#if __cplusplus >= 201103L
  std::unique_ptr<xn::ImageGenerator> imagegen_uniqueptr(image_gen_);
#else
  std::auto_ptr<xn::ImageGenerator> imagegen_uniqueptr(image_gen_);
#endif

  XnStatus st;

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_IMAGE, image_gen_);

  fawkes::openni::setup_map_generator(*image_gen_, config);

  fawkes::openni::get_usb_info(*image_gen_, usb_vendor_, usb_product_);

  if ( (usb_vendor_ == 0x045e) && (usb_product_ == 0x02ae) ) {
    // from OpenNI-PrimeSense/XnStreamParams.h:
    // XN_IO_IMAGE_FORMAT_UNCOMPRESSED_BAYER = 6
    // InputFormat should be 6 = uncompressed Bayer for Kinect
    logger->log_debug(name(), "Kinect camera detected, initializing");
    if (image_gen_->SetIntProperty("InputFormat", 6) != XN_STATUS_OK) {
      throw Exception("Failed to set uncompressed bayer input format");
    }
    if (image_gen_->SetPixelFormat(XN_PIXEL_FORMAT_GRAYSCALE_8_BIT) != XN_STATUS_OK)
    {
      throw Exception("Failed to set pixel format");
    }
    /*
    // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
    // (from ROS openni_camera)
    if (depth_gen_->SetIntProperty ("RegistrationType", 2) != XN_STATUS_OK) {
      throw Exception("Failed to set registration type");
    }
    */
    cfg_copy_mode_ = DEBAYER_BILINEAR;
    try {
      std::string debayering = config->get_string("/plugins/openni-image/debayering");
      if (debayering == "bilinear") {
        cfg_copy_mode_ = DEBAYER_BILINEAR;
      } else if (debayering == "nearest_neighbor") {
        cfg_copy_mode_ = DEBAYER_NEAREST_NEIGHBOR;
      } else {
        logger->log_warn(name(), "Unknown de-bayering mode '%s', using bilinear instead.",
			 debayering.c_str());
      }
    } catch (Exception &e) {
      logger->log_warn(name(), "No de-bayering mode set, using bilinear.");
    }
  } else {
    logger->log_debug(name(), "PrimeSense camera detected, initializing");
    if (image_gen_->SetIntProperty("InputFormat", 5) != XN_STATUS_OK) {
      throw Exception("Failed to set uncompressed bayer input format");
    }
    if (image_gen_->SetPixelFormat(XN_PIXEL_FORMAT_YUV422) != XN_STATUS_OK) {
      throw Exception("Failed to set pixel format");
    }
    cfg_copy_mode_ = CONVERT_YUV;
  }

  image_md_ = new xn::ImageMetaData();

  image_gen_->GetMetaData(*image_md_);

  image_width_  = image_md_->XRes();
  image_height_ = image_md_->YRes();

  /*
  const char *pixel_format = "unknown";
  switch (image_gen_->GetPixelFormat()) {
  case XN_PIXEL_FORMAT_RGB24:            pixel_format = "RGB24"; cfg_copy_mode_ = CONVERT_RGB; break;
  case XN_PIXEL_FORMAT_YUV422:           pixel_format = "YUV422"; break;
  case XN_PIXEL_FORMAT_GRAYSCALE_8_BIT:  pixel_format = "Gray8"; break;
  case XN_PIXEL_FORMAT_GRAYSCALE_16_BIT: pixel_format = "Gray16"; break; 	
  case XN_PIXEL_FORMAT_MJPEG:            pixel_format = "MJPEG"; break; 	
  }

  XnUInt64 input_format;
  if (image_gen_->GetIntProperty("InputFormat", input_format) != XN_STATUS_OK) {
    logger->log_warn(name(), "Failed to get input format");
  }

  logger->log_debug(name(), "Image  format: %s  width: %u  height: %u  input format: %lu",
		    pixel_format, image_md_->XRes(), image_md_->YRes(), input_format);
  */

  image_buf_yuv_ =
    new SharedMemoryImageBuffer("openni-image-yuv", YUV422_PLANAR,
                                image_md_->XRes(), image_md_->YRes());

  image_buf_rgb_ =
    new SharedMemoryImageBuffer("openni-image-rgb", RGB,
                                image_md_->XRes(), image_md_->YRes());


  image_gen_->StartGenerating();

  capture_start_ = new Time(clock);
  capture_start_->stamp_systime();
  // Update once to get timestamp
  image_gen_->WaitAndUpdateData();
  // arbitrarily define the zero reference point,
  // we can't get any closer than this
  *capture_start_ -= (long int)image_gen_->GetTimestamp();
  
  imagegen_uniqueptr.release();
}


void
OpenNiImageThread::finalize()
{
  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete image_gen_;
  delete image_md_;
  delete image_buf_yuv_;
  delete image_buf_rgb_;
}


void
OpenNiImageThread::loop()
{
  MutexLocker lock(openni.objmutex_ptr());
  bool is_image_new = image_gen_->IsDataNew();
  image_gen_->GetMetaData(*image_md_);
  const XnUInt8 * const      image_data = image_md_->Data();
  fawkes::Time ts = *capture_start_ + (long int)image_gen_->GetTimestamp();
  lock.unlock();

  if (is_image_new && (image_buf_yuv_->num_attached() > 1)) {
    image_buf_yuv_->lock_for_write();
    if (cfg_copy_mode_ == DEBAYER_BILINEAR) {
      bayerGRBG_to_yuv422planar_bilinear(image_data, image_buf_yuv_->buffer(),
					 image_width_, image_height_);
    } else if (cfg_copy_mode_ == CONVERT_YUV) {
      yuv422packed_to_yuv422planar(image_data, image_buf_yuv_->buffer(),
				   image_width_, image_height_);
    } else if (cfg_copy_mode_ == CONVERT_RGB) {
      rgb_to_yuv422planar_plainc(image_data, image_buf_yuv_->buffer(),
				 image_width_, image_height_);
    } else if (cfg_copy_mode_ == DEBAYER_NEAREST_NEIGHBOR) {
      bayerGRBG_to_yuv422planar_nearest_neighbour(image_data,
						  image_buf_yuv_->buffer(),
						  image_width_, image_height_);
    }
    image_buf_yuv_->set_capture_time(&ts);
    image_buf_yuv_->unlock();
  }

  if (is_image_new && (image_buf_rgb_->num_attached() > 1)) {
    image_buf_rgb_->lock_for_write();
    if (cfg_copy_mode_ == DEBAYER_BILINEAR) {
      bayerGRBG_to_rgb_bilinear(image_data, image_buf_rgb_->buffer(),
                                image_width_, image_height_);
    } else if (cfg_copy_mode_ == CONVERT_YUV) {
      yuv422packed_to_rgb_plainc(image_data, image_buf_rgb_->buffer(),
                                 image_width_, image_height_);
    } else if (cfg_copy_mode_ == CONVERT_RGB) {
      memcpy(image_buf_rgb_->buffer(), image_data,
	     colorspace_buffer_size(RGB, image_width_, image_height_));
    } else if (cfg_copy_mode_ == DEBAYER_NEAREST_NEIGHBOR) {
      bayerGRBG_to_rgb_nearest_neighbour(image_data, image_buf_rgb_->buffer(),
                                         image_width_, image_height_);
    }
    image_buf_rgb_->set_capture_time(&ts);
    image_buf_rgb_->unlock();
  }
}
