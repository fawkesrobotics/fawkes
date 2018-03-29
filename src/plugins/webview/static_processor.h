
/***************************************************************************
 *  static_processor.h - Web request processor for static files
 *
 *  Created: Mon Oct 13 23:41:34 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WEBVIEW_STATIC_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_STATIC_PROCESSOR_H_

#include <webview/request_processor.h>
#include <cstdlib>
#include <vector>

namespace fawkes {
  class Logger;
}

class WebviewStaticRequestProcessor : public fawkes::WebRequestProcessor
{
 public:
  WebviewStaticRequestProcessor(const char *baseurl,
				std::vector<const char *> htdocs_dir,
				fawkes::Logger *logger);
  virtual ~WebviewStaticRequestProcessor();

  virtual fawkes::WebReply * process_request(const fawkes::WebRequest *request);

 private:
  char   *baseurl_;
  size_t  baseurl_len_;
  std::vector<char*> htdocs_dirs_;
  std::vector<size_t> htdocs_dirs_len_;

  fawkes::Logger *logger_;
};

#endif
