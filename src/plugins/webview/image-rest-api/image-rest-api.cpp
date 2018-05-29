
/***************************************************************************
 *  image-rest-api.cpp - Image REST API
 *
 *  Created: Sat Apr 07 23:06:51 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "image-rest-api.h"

#include "jpeg_stream_producer.h"
#include "mjpeg_reply.h"

#include <webview/rest_api_manager.h>
#include <fvutils/ipc/shm_image.h>

using namespace fawkes;
using namespace firevision;

/** @class ImageRestApi "skiller-rest-api.h"
 * REST API backend for the image.
 * @author Tim Niemueller
 */

/** Constructor. */
ImageRestApi::ImageRestApi()
	: Thread("ImageRestApi", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
ImageRestApi::~ImageRestApi()
{
}

void
ImageRestApi::init()
{
	rest_api_ = new WebviewRestApi("images", logger);
	rest_api_->add_handler<WebviewRestArray<ImageInfo>>
		(WebRequest::METHOD_GET, "/?",
		 std::bind(&ImageRestApi::cb_list_images, this));
	rest_api_->add_handler(WebRequest::METHOD_GET, "/{id+}",
	                       std::bind(&ImageRestApi::cb_get_image, this, std::placeholders::_1));
	webview_rest_api_manager->register_api(rest_api_);
}

void
ImageRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
  for (auto &s : streams_) {
	  thread_collector->remove(&*s.second);
  }
  streams_.clear();
}


void
ImageRestApi::loop()
{
}


WebviewRestArray<ImageInfo>
ImageRestApi::cb_list_images()
{
	WebviewRestArray<ImageInfo> rv;

	std::list<SharedMemoryImageBufferMetaData> meta_data =
		SharedMemoryImageBuffer::list_meta_data();

	for (auto &m : meta_data) {
		ImageInfo image;
		image.set_kind("ImageInfo");
		image.set_apiVersion(ImageInfo::api_version());
		image.set_id(m.image_id);
		image.set_colorspace(colorspace_to_string(m.colorspace));
		image.set_frame(m.frame_id);
		image.set_width(m.width);
		image.set_height(m.height);
		image.set_mem_size(m.mem_size);
		rv.push_back(std::move(image));
	}

	return rv;
}

std::shared_ptr<fawkes::WebviewJpegStreamProducer>
ImageRestApi::get_stream(const std::string& image_id)
{
	if (streams_.find(image_id) == streams_.end()) {
    try {
	    std::string cfg_prefix = "/webview/images/" + image_id + "/";
	    unsigned int quality = 80;
	    float fps = 15;
	    bool vflip = false;
	    // Read default values if set
	    try {
		    quality = config->get_uint("/webview/images/default/jpeg-quality");
      } catch (Exception &e) {} // ignored, use default
	    try {
		    fps = config->get_float("/webview/images/default/mjpeg-fps");
	    } catch (Exception &e) {} // ignored, use default
	    try {
		    vflip = config->get_bool("/webview/images/default/jpeg-vflip");
	    } catch (Exception &e) {} // ignored, use default
	    // Set camera-specific values
	    try {
		    quality = config->get_uint((cfg_prefix + "jpeg-quality").c_str());
	    } catch (Exception &e) {} // ignored, use default
	    try {
		    fps = config->get_float((cfg_prefix + "mjpeg-fps").c_str());
	    } catch (Exception &e) {} // ignored, use default
	    try {
		    vflip = config->get_bool((cfg_prefix + "jpeg-vflip").c_str());
	    } catch (Exception &e) {} // ignored, use default

	    auto stream =
		    std::make_shared<WebviewJpegStreamProducer>(image_id, quality, fps, vflip);

      thread_collector->add(&*stream);

      streams_[image_id] = stream;
    } catch (Exception &e) {
	    logger->log_warn("ImageRestApi", "Failed to open buffer '%s',"
	                     " exception follows", image_id.c_str());
	    logger->log_warn("ImageRestApi", e);
      return NULL;
    }
  }

  return streams_[image_id];
}

std::unique_ptr<WebReply>
ImageRestApi::cb_get_image(WebviewRestParams& params)
{
	std::string image   = params.path_arg("id");

	std::string::size_type last_dot = image.rfind(".");
	if (last_dot == std::string::npos) {
		return std::make_unique<StaticWebReply>(WebReply::HTTP_NOT_FOUND, "Invalid stream ID");
	}
	std::string image_id = image.substr(0, last_dot);
	std::string image_type = image.substr(last_dot + 1);

	std::shared_ptr<WebviewJpegStreamProducer> stream = get_stream(image_id);
	if (! stream) {
		return std::make_unique<StaticWebReply>(WebReply::HTTP_NOT_FOUND, "Stream not found");
	}

	if (image_type == "jpeg" || image_type == "jpg") {
		std::shared_ptr<WebviewJpegStreamProducer::Buffer> buf = stream->wait_for_next_frame();

		//logger_->log_debug("WebImageReqProc", "Compressed buffer size: %zu", buf->size());
		std::string body((char *)buf->data(), buf->size());
		auto reply = std::make_unique<StaticWebReply>(WebReply::HTTP_OK, body);
		reply->add_header("Content-type", "image/jpeg");
		reply->set_caching(false);
		return reply;
	} else if (image_type == "mjpeg" || image_type == "mjpg") {
		return std::make_unique<DynamicMJPEGStreamWebReply>(stream);
	} else {
		return std::make_unique<StaticWebReply>(WebReply::HTTP_NOT_FOUND, "Unknown image format");
	}
}
