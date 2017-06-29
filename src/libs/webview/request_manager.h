
/***************************************************************************
 *  request_manager.h - Web Requestigation manager
 *
 *  Created: Tue Dec 21 01:28:50 2010
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

#ifndef __LIBS_WEBVIEW_REQUEST_MANAGER_H_
#define __LIBS_WEBVIEW_REQUEST_MANAGER_H_

#include <memory>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Time;
class Mutex;
class WebServer;

class WebRequestManager
{
  friend WebServer;
 public:
  WebRequestManager();
  ~WebRequestManager();

  unsigned int num_active_requests() const;
  Time last_request_completion_time() const;

 private:
  void set_server(WebServer *server);

 private:
  Mutex     *mutex_;
  WebServer *server_;
};

} // end namespace fawkes

#endif
