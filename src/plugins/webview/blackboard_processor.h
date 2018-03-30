
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

#include <map>
#include <string>

namespace fawkes {
  class BlackBoard;
  class Interface;
  class WebUrlManager;
  class WebReply;
  class WebRequest;
}

class WebviewBlackBoardRequestProcessor
{
 public:
  WebviewBlackBoardRequestProcessor(fawkes::WebUrlManager *url_manager,
                                    fawkes::BlackBoard *blackboard);
  ~WebviewBlackBoardRequestProcessor();

 private:
#if defined(HAVE_GRAPHVIZ) && ((defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 5))) || defined(__clang__))
  std::string generate_graph(std::string for_owner = "");
#endif

  fawkes::WebReply * process_graph(const fawkes::WebRequest *request);
  fawkes::WebReply * process_graphview(const fawkes::WebRequest *request);
  fawkes::WebReply * process_view(const fawkes::WebRequest *request);
  fawkes::WebReply * process_overview();

 private:
  fawkes::BlackBoard *blackboard_;
  fawkes::WebUrlManager *url_manager_;

  std::map<std::string, fawkes::Interface *> interfaces_;
  std::map<std::string, fawkes::Interface *>::iterator ifi_;
};

#endif
