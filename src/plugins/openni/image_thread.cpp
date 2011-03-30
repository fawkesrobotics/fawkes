
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

#include <memory>

using namespace fawkes;
using namespace firevision;

/** @class OpenNiImageThread "image_thread.h"
 * OpenNI Image Provider Thread.
 * This thread provides RGB and depth images from the camera via a
 * SharedMemoryImageBuffer to other FireVision plugins.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiImageThread::OpenNiImageThread()
  : Thread("OpenNiImageThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
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
  std::auto_ptr<xn::ImageGenerator> imagegen_autoptr(__image_gen);

  __depth_gen = new xn::DepthGenerator();
  std::auto_ptr<xn::DepthGenerator> depthgen_autoptr(__depth_gen);

  XnStatus st;

  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_IMAGE, __image_gen);
  fawkes::openni::find_or_create_node(openni, XN_NODE_TYPE_DEPTH, __depth_gen);

  fawkes::openni::setup_map_generator(*__image_gen, config);
  fawkes::openni::setup_map_generator(*__depth_gen, config);

  __usb_vendor = 0;
  __usb_product = 0;

  xn::NodeInfo image_info = __image_gen->GetInfo();
  xn::NodeInfoList &depnodes = image_info.GetNeededNodes();
  for (xn::NodeInfoList::Iterator n = depnodes.Begin(); n != depnodes.End(); ++n) {
    const XnProductionNodeDescription &pnd = (*n).GetDescription();

    if ((pnd.Type == XN_NODE_TYPE_DEVICE) &&
        (strcmp(pnd.strVendor, "PrimeSense") == 0) &&
	(strcmp(pnd.strName, "SensorV2") == 0) )
    {
      // it's the primesense device node and we can check for USB vendor/product
      unsigned short int vendor = 0, product = 0;
      unsigned char bus = 0, addr = 0;
      if (sscanf((*n).GetCreationInfo(), "%04hx/%04hx@%hhu/%hhu", &vendor, &product, &bus, &addr) == 4) {
	logger->log_debug(name(), "Detected USB device (vendor: %04hx  product: %04hx  bus: %hhu  addr: %hhu)",
			  vendor, product, bus, addr);
	__usb_vendor  = vendor;
	__usb_product = product;
      }
    }
  }

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
    /*
    // RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
    // (from ROS openni_camera)
    if (__depth_gen->SetIntProperty ("RegistrationType", 1) != XN_STATUS_OK) {
      throw Exception("Failed to set registration type");
    }
    */
    __cfg_copy_mode = CONVERT_YUV;
  }

  __image_md = new xn::ImageMetaData();
  __depth_md = new xn::DepthMetaData();

  __image_gen->GetMetaData(*__image_md);
  __depth_gen->GetMetaData(*__depth_md);

  __image_width  = __image_md->XRes();
  __image_height = __image_md->YRes();
  __depth_width  = __image_md->XRes();
  __depth_height = __image_md->YRes();

  /*
  const char *pixel_format = "unknown";
  switch (__image_gen->GetPixelFormat()) {
  case XN_PIXEL_FORMAT_RGB24:            pixel_format = "RGB24"; break;
  case XN_PIXEL_FORMAT_YUV422:           pixel_format = "YUV422"; break;
  case XN_PIXEL_FORMAT_GRAYSCALE_8_BIT:  pixel_format = "Gray8"; break;
  case XN_PIXEL_FORMAT_GRAYSCALE_16_BIT: pixel_format = "Gray16"; break; 	
  }

  XnUInt64 input_format;
  if (__image_gen->GetIntProperty("InputFormat", input_format) != XN_STATUS_OK) {
    logger->log_warn(name(), "Failed to get input format");
  }

  logger->log_debug(name(), "Image  format: %s  width: %u  height: %u  input format: %lu",
		    pixel_format, __image_md->XRes(), __image_md->YRes(), input_format);
  */

  __image_buf = new SharedMemoryImageBuffer("openni-image", YUV422_PLANAR,
					    __image_md->XRes(),
					    __image_md->YRes());

  __depth_buf = new SharedMemoryImageBuffer("openni-depth", RAW16,
  					    __depth_md->XRes(), __depth_md->YRes());
  __depth_bufsize = colorspace_buffer_size(RAW16,
					   __depth_md->XRes(), __depth_md->YRes());

  __image_gen->StartGenerating();
  __depth_gen->StartGenerating();

  imagegen_autoptr.release();
  depthgen_autoptr.release();
}


void
OpenNiImageThread::finalize()
{
  // we do not stop generating, we don't know if there is no other plugin
  // using the node.
  delete __image_gen;
  delete __depth_gen;

  delete __image_md;
  delete __depth_md;

  delete __image_buf;
  delete __depth_buf;
}


void
OpenNiImageThread::loop()
{
  MutexLocker lock(openni.objmutex_ptr());
  bool is_image_new = __image_gen->IsDataNew();
  bool is_depth_new = __depth_gen->IsDataNew();
  const XnUInt8 * const      image_data = __image_md->Data();
  const XnDepthPixel * const depth_data = __depth_md->Data();
  lock.unlock();

  if (is_image_new && (__image_buf->num_attached() > 1)) {
    if (__cfg_copy_mode == DEBAYER_BILINEAR) {
      bayerGRBG_to_yuv422planar_bilinear(image_data, __image_buf->buffer(),
					 __image_width, __image_height);
    } else if (__cfg_copy_mode == CONVERT_YUV) {
      yuv422packed_to_yuv422planar(image_data, __image_buf->buffer(),
				   __image_width, __image_height);
    } else if (__cfg_copy_mode == DEBAYER_NEAREST_NEIGHBOR) {
      bayerGRBG_to_yuv422planar_nearest_neighbour(image_data,
						  __image_buf->buffer(),
						  __image_width, __image_height);
    }
  }

  if (is_depth_new && (__depth_buf->num_attached() > 1)) {
    memcpy(__depth_buf->buffer(), depth_data, __depth_bufsize);
  }
}
