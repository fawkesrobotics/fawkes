
/***************************************************************************
 *  camera_processor.h - Web request processor for viewing camera images
 *
 *  Created: Wed Feb 05 17:43:36 2014
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

#ifndef __PLUGINS_WEBVIEW_CAMERA_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_CAMERA_PROCESSOR_H_

#include <webview/request_processor.h>
#include <string>

namespace fawkes {
  class Logger;
}

namespace firevision {
  class SharedMemoryCamera;
  class JpegImageCompressor;
}

class WebviewCameraRequestProcessor : public fawkes::WebRequestProcessor
{
 public:
  WebviewCameraRequestProcessor(const char *baseurl,
				fawkes::Logger *logger);
  virtual ~WebviewCameraRequestProcessor();

  virtual fawkes::WebReply * process_request(const fawkes::WebRequest *request);

 private:
  firevision::SharedMemoryCamera * get_camera(const std::string &image_id);


 private:
  char   *baseurl_;
  size_t  baseurl_len_;

  fawkes::Logger *logger_;

  firevision::JpegImageCompressor *jpeg_;
  std::map<std::string, firevision::SharedMemoryCamera *> cams_;
};

#endif
