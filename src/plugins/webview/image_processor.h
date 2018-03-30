
/***************************************************************************
 *  image_processor.h - Web request processor for viewing images
 *
 *  Created: Wed Feb 05 17:43:36 2014
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

#ifndef __PLUGINS_WEBVIEW_IMAGE_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_IMAGE_PROCESSOR_H_

#include <string>
#include <map>

namespace fawkes {
  class Configuration;
  class Logger;
  class ThreadCollector;
  class WebviewJpegStreamProducer;
  class WebReply;
  class WebRequest;
  class WebUrlManager;
}

class WebviewImageRequestProcessor
{
 public:
	WebviewImageRequestProcessor(fawkes::WebUrlManager *url_manager, fawkes::Configuration *config,
                               fawkes::Logger *logger, fawkes::ThreadCollector *thread_col);
  ~WebviewImageRequestProcessor();

 private:
  fawkes::WebReply * process_image(const fawkes::WebRequest *request);
  fawkes::WebReply * process_overview();

  fawkes::WebviewJpegStreamProducer * get_stream(const std::string &image_id);

 private:
  fawkes::Configuration *config_;
  fawkes::Logger *logger_;
  fawkes::ThreadCollector *thread_col_;
  fawkes::WebUrlManager *url_manager_;

  std::map<std::string, fawkes::WebviewJpegStreamProducer *> streams_;
};

#endif
