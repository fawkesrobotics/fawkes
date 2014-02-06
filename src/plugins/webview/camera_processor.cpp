
/***************************************************************************
 *  camera_processor.cpp - Web request processor for camera images/streams
 *
 *  Created: Wed Feb 05 17:48:34 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "camera_processor.h"
#include "mjpeg_reply.h"
#include <webview/file_reply.h>
#include <webview/error_reply.h>
#include <fvutils/compression/jpeg_compressor.h>
#include <fvcams/shmem.h>

#include <core/exception.h>
#include <logging/logger.h>

#include <cstring>
#include <cstdlib>
#include <string>

using namespace fawkes;
using namespace firevision;

/** @class WebviewCameraRequestProcessor "mjpeg_processor.h"
 * Camera stream web processor.
 * This processor provides access to image buffers on the system as
 * Camera streams.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param baseurl Base URL where the static processor is mounted
 * @param logger logger
 */
WebviewCameraRequestProcessor::WebviewCameraRequestProcessor(const char *baseurl,
							     fawkes::Logger *logger)
{
  logger_         = logger;
  baseurl_        = strdup(baseurl);
  baseurl_len_    = strlen(baseurl_);

  jpeg_ = new JpegImageCompressor(15);
}

/** Destructor. */
WebviewCameraRequestProcessor::~WebviewCameraRequestProcessor()
{
  free(baseurl_);
  delete jpeg_;
}


SharedMemoryCamera *
WebviewCameraRequestProcessor::get_camera(const std::string &image_id)
{
  if (cams_.find(image_id) == cams_.end()) {
    try {
      SharedMemoryCamera *cam =
	new SharedMemoryCamera(image_id.c_str(), /* deep copy */ true);

      cams_[image_id] = cam;
    } catch (Exception &e) {
      logger_->log_warn("WebCameraReqProc", "Failed to open buffer '%s',"
			" exception follows", image_id.c_str());
      logger_->log_warn("WebCameraReqProc", e);
      return NULL;
    }
  }

  return cams_[image_id];
}


WebReply *
WebviewCameraRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( strncmp(baseurl_, request->url().c_str(), baseurl_len_) == 0 ) {
    // It is in our URL prefix range
    std::string subpath = request->url().substr(baseurl_len_);

    if (subpath.find("/view/") == 0) {
      // still image

      std::string::size_type last_dot = subpath.rfind(".");
      if (last_dot == std::string::npos) {
	return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Invalid stream ID");
      }
      std::string image_id = subpath.substr(6, last_dot - 6);
      std::string image_type = subpath.substr(last_dot + 1);
      logger_->log_info("***", "ID: %s  Type: %s", image_id.c_str(), image_type.c_str());

      SharedMemoryCamera *cam = get_camera(image_id);
      if (cam == 0) {
	return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Stream not found");
      }

      if (image_type == "jpeg" || image_type == "jpg") {
	cam->capture();

	//jpeg_->set_filename("/tmp/webviewstill.jpg");
	jpeg_->set_image_dimensions(cam->pixel_width(), cam->pixel_height());
	jpeg_->set_image_buffer(cam->colorspace(), cam->buffer());
	jpeg_->set_compression_destination(ImageCompressor::COMP_DEST_MEM);

	size_t size = jpeg_->recommended_compressed_buffer_size();
	unsigned char *compressed = (unsigned char *)malloc(size);
	jpeg_->set_destination_buffer(compressed, size);
	jpeg_->compress();

	cam->dispose_buffer();

	logger_->log_debug("WebCameraReqProc", "Compressed buffer size: %zu",
			   jpeg_->compressed_size());
	//return new DynamicFileWebReply("/tmp/webviewstill.jpg");
	std::string body((char *)compressed, jpeg_->compressed_size());
	free(compressed);
	StaticWebReply *reply = new StaticWebReply(WebReply::HTTP_OK, body);
	reply->add_header("Content-type", "image/jpeg");
	reply->set_caching(false);
	return reply;

      } else if (image_type == "mjpeg") {
	return new DynamicMJPEGStreamWebReply(cam);
      } else {
	return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Unknown image format");
      }
    } else {
      return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Unknown request");
    }
  } else {
    // wrong base url, why the heck are we called!?
    logger_->log_error("WebCameraReqProc", "Called for invalid base url "
		       "(url: %s, baseurl: %s)", request->url().c_str(), baseurl_);
    return NULL;
  }
}
