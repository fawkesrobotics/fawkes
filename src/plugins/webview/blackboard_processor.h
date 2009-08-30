
/***************************************************************************
 *  blackboard_processor.h - Web request processor for BlackBoard info
 *
 *  Created: Thu Oct 23 16:08:10 2008
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

#ifndef __PLUGINS_WEBVIEW_BLACKBOARD_PROCESSOR_H_
#define __PLUGINS_WEBVIEW_BLACKBOARD_PROCESSOR_H_

#include <webview/request_processor.h>

#include <map>
#include <string>

namespace fawkes {
  class BlackBoard;
  class Interface;
}

class WebviewBlackBoardRequestProcessor : public WebRequestProcessor
{
 public:
  WebviewBlackBoardRequestProcessor(const char *baseurl,
				fawkes::BlackBoard *blackboard);
  virtual ~WebviewBlackBoardRequestProcessor();

  virtual WebReply * process_request(const char *url,
				     const char *method,
				     const char *version,
				     const char *upload_data,
				     size_t *upload_data_size,
				     void **session_data);

 private:
  char *__baseurl;
  size_t __baseurl_len;
  fawkes::BlackBoard *__blackboard;

  std::map<std::string, fawkes::Interface *> __interfaces;
  std::map<std::string, fawkes::Interface *>::iterator __ifi;

};

#endif
