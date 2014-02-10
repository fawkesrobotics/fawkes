
/***************************************************************************
 *  image_processor.cpp - Web request processor for images/streams
 *
 *  Created: Wed Feb 05 17:48:34 2014
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include "image_processor.h"
#include "mjpeg_reply.h"
#include "jpeg_stream_producer.h"
#include <webview/file_reply.h>
#include <webview/error_reply.h>

#include <core/exception.h>
#include <core/threading/thread_collector.h>
#include <config/config.h>
#include <logging/logger.h>
#include <fvutils/ipc/shm_image.h>

#include <cstring>
#include <cstdlib>
#include <string>

using namespace fawkes;
using namespace firevision;

/** @class WebviewImageRequestProcessor "image_processor.h"
 * Image stream web processor.
 * This processor provides access to image buffers on the system as
 * Image streams.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param baseurl Base URL where the static processor is mounted
 * @param config system configuration
 * @param logger logger
 * @param thread_col thread collector to use for stream producers
 */
WebviewImageRequestProcessor::WebviewImageRequestProcessor(const char *baseurl,
							   fawkes::Configuration *config,
							   fawkes::Logger *logger,
							   fawkes::ThreadCollector *thread_col)
{
  config_         = config;
  logger_         = logger;
  thread_col_     = thread_col;
  baseurl_        = strdup(baseurl);
  baseurl_len_    = strlen(baseurl_);
}

/** Destructor. */
WebviewImageRequestProcessor::~WebviewImageRequestProcessor()
{
  free(baseurl_);
  for (auto s : streams_) {
    thread_col_->remove(s.second);
    delete s.second;
  }
}

WebviewJpegStreamProducer *
WebviewImageRequestProcessor::get_stream(const std::string &image_id)
{
  if (streams_.find(image_id) == streams_.end()) {
    try {
      std::string cfg_prefix = "/webview/images/" + image_id + "/";
      unsigned int quality = 80;
      float fps = 15;
      bool vflip = false;
      // Read default values if set
      try {
	quality = config_->get_uint("/webview/images/default/jpeg-quality");
      } catch (Exception &e) {} // ignored, use default
      try {
	fps = config_->get_float("/webview/images/default/mjpeg-fps");
      } catch (Exception &e) {} // ignored, use default
      try {
	vflip = config_->get_bool("/webview/images/default/jpeg-vflip");
      } catch (Exception &e) {} // ignored, use default
      // Set camera-specific values
      try {
	quality = config_->get_uint((cfg_prefix + "jpeg-quality").c_str());
      } catch (Exception &e) {} // ignored, use default
      try {
	fps = config_->get_float((cfg_prefix + "mjpeg-fps").c_str());
      } catch (Exception &e) {} // ignored, use default
      try {
	vflip = config_->get_bool((cfg_prefix + "jpeg-vflip").c_str());
      } catch (Exception &e) {} // ignored, use default

      WebviewJpegStreamProducer *stream =
	new WebviewJpegStreamProducer(image_id.c_str(), quality, fps, vflip);

      thread_col_->add(stream);

      streams_[image_id] = stream;
    } catch (Exception &e) {
      logger_->log_warn("WebImageReqProc", "Failed to open buffer '%s',"
			" exception follows", image_id.c_str());
      logger_->log_warn("WebImageReqProc", e);
      return NULL;
    }
  }

  return streams_[image_id];
}


WebReply *
WebviewImageRequestProcessor::process_request(const fawkes::WebRequest *request)
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

      WebviewJpegStreamProducer *stream = get_stream(image_id);
      if (! stream) {
	return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Stream not found");
      }

      if (image_type == "jpeg" || image_type == "jpg") {
	RefPtr<WebviewJpegStreamProducer::Buffer> buf = stream->wait_for_next_frame();

	//logger_->log_debug("WebImageReqProc", "Compressed buffer size: %zu", buf->size());
	std::string body((char *)buf->data(), buf->size());
	StaticWebReply *reply = new StaticWebReply(WebReply::HTTP_OK, body);
	reply->add_header("Content-type", "image/jpeg");
	reply->set_caching(false);
	return reply;
      } else if (image_type == "mjpeg" || image_type == "mjpg") {
	return new DynamicMJPEGStreamWebReply(stream);
      } else {
	return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Unknown image format");
      }
    } else if (subpath == "" || subpath == "/") {
      // list all buffers

      WebPageReply *r = new WebPageReply("Image Buffers");
      std::list<SharedMemoryImageBufferMetaData> meta_data =
	SharedMemoryImageBuffer::list_meta_data();

      if (meta_data.empty()) {
	*r += "<p><b>No image buffers found.</b></p>\n";
      } else {
	*r += "<h2>Image Buffers</h2>\n";
        *r += "<table>\n";
        *r += "<tr><th>Buffer</th><th>Frame</th><th>Colorspace</th>"
	  "<th>Dimensions</th><th>Memory</th><th>View as</th></tr>\n";
	for (auto m : meta_data) {
	  r->append_body("<tr><td>%s</td><td>%s</td><td>%s</td>"
			 "<td>%ux%u</td><td>%zu B</td>"
			 "<td><div class=\"actionlist\"><ul><li><a href=\"%s/view/%s.jpg\">JPEG</a></li>"
			 "<li><a href=\"%s/view/%s.mjpeg\">Stream</a></li></ul></div></td>"
			 "</tr>\n", m.image_id.c_str(), m.frame_id.c_str(),
			 colorspace_to_string(m.colorspace), m.width, m.height,
			 m.mem_size, baseurl_, m.image_id.c_str(), baseurl_, m.image_id.c_str());
	}
	*r += "</table>\n";
      }

      return r;

    } else {
      return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "Unknown request");
    }
  } else {
    // wrong base url, why the heck are we called!?
    logger_->log_error("WebImageReqProc", "Called for invalid base url "
		       "(url: %s, baseurl: %s)", request->url().c_str(), baseurl_);
    return NULL;
  }
}
