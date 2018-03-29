
/***************************************************************************
 *  url_manager.h - Web URL manager
 *
 *  Created: Thu Nov 25 21:53:07 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_WEBVIEW_URL_MANAGER_H_
#define __LIBS_WEBVIEW_URL_MANAGER_H_

#include <map>
#include <string>
#include <algorithm>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Mutex;
class WebRequestProcessor;

class WebUrlManager
{
 public:
  WebUrlManager();
  ~WebUrlManager();

  void register_baseurl(const char *url_prefix, WebRequestProcessor *processor);
  void unregister_baseurl(const char *url_prefix);

  WebRequestProcessor * find_processor(const std::string& url) const;
  Mutex * mutex();

 private:
  Mutex                                        *mutex_;
  std::map<std::string, WebRequestProcessor *, std::greater<std::string>> processors_;
};

} // end namespace fawkes

#endif
