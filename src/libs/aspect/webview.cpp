
/***************************************************************************
 *  webview.cpp - Webview aspect for Fawkes
 *
 *  Created: Thu Nov 25 22:20:52 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <aspect/webview.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebviewAspect <aspect/webview.h>
 * Thread aspect to provide web pages via Webview.
 *
 * The WebviewAspect differs from other aspects. It can be successfully
 * initialized even if there currently is no plugin loaded to serve web
 * pages. This is because the central interface, the WebUrlManager, is
 * held by the WebviewAspectIniFin and thus processor can be registerd
 * and deregistered all the time. The webview plugin itself has the
 * WebviewAspect, but it uses its access to the WebUrlManager instance
 * to serve requests and pass them on to the appropriate processor.
 *
 * It is guaranteed that if used properly from within plugins that
 * init_WebviewAspect() is called before the thread is started and that
 * you can access the webview request processor manager in the thread's
 * init() method.
 *
 * @ingroup Aspects
 * @author Tim Niemueller
 */


/** Constructor. */
WebviewAspect::WebviewAspect()
{
  add_aspect("WebviewAspect");
}


/** Virtual empty Destructor. */
WebviewAspect::~WebviewAspect()
{
}


/** Set URL manager.
 * It is guaranteed that this is called for a logging thread before
 * Thread::start() is called (when running regularly inside Fawkes).
 * @param url_manager URL manager to register processors to
 * @param nav_manager Navigation manager to add navigation entries
 * @param request_manager Request manager to query request information
 * @param rest_api_manager Manager to register REST APIs
 * @see WebviewMaster
 */
void
WebviewAspect::init_WebviewAspect(WebUrlManager *url_manager,
                                  WebNavManager *nav_manager,
                                  WebRequestManager *request_manager,
                                  WebviewRestApiManager *rest_api_manager)
{
  webview_url_manager = url_manager;
  webview_nav_manager = nav_manager;
  webview_request_manager = request_manager;
  webview_rest_api_manager = rest_api_manager;
}

} // end namespace fawkes
