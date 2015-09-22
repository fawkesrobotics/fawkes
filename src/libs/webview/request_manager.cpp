
/***************************************************************************
 *  request_manager.cpp - Web Request manager
 *
 *  Created: Fri Feb 07 16:52:20 2014
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

#include <webview/request_manager.h>
#include <core/threading/mutex_locker.h>
#include <webview/server.h>
#include <utils/time/time.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** @class WebRequestManager <webview/nav_manager.h>
 * Probides information about ongoing requests.
 * Will take a server at run-time and query it for request information.
 * This class can persists even though the server does not, which is
 * required fr the WebviewAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
WebRequestManager::WebRequestManager()
{
  mutex_  = new Mutex();
  server_ = 0;
}


/** Destructor. */
WebRequestManager::~WebRequestManager()
{
  delete mutex_;
}


void
WebRequestManager::set_server(WebServer *server)
{
  MutexLocker lock(mutex_);
  server_ = server;
}


/** Get number of currently active requests.
 * @return number of currently active requests.
 */
unsigned int
WebRequestManager::num_active_requests() const
{
  MutexLocker lock(mutex_);
  if (server_) {
    return server_->active_requests();
  } else {
    return 0;
  }
}


/** Get time when last request was completed.
 * If the number of active requests is zero this gives the time of
 * last activity. Otherwise just says when the last request was
 * completed.
 * @return time when last request was completed
 */
Time
WebRequestManager::last_request_completion_time() const
{
  MutexLocker lock(mutex_);
  if (server_) {
    return server_->last_request_completion_time();
  } else {
    return Time(0,0);
  }
}


} // end namespace fawkes
