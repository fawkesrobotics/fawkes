
/***************************************************************************
 *  webview.h - Webview aspect for Fawkes
 *
 *  Created: Thu Nov 25 22:19:10 2010
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

#ifndef __ASPECT_WEBVIEW_H_
#define __ASPECT_WEBVIEW_H_

#include <aspect/aspect.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebUrlManager;
class WebNavManager;
class WebRequestManager;
class WebviewRestApiManager;

class WebviewAspect : public virtual Aspect
{
 public:
  WebviewAspect();
  virtual ~WebviewAspect();

  void              init_WebviewAspect(WebUrlManager *url_manager,
                                       WebNavManager *nav_manager,
                                       WebRequestManager *request_manager,
                                       WebviewRestApiManager *rest_api_manager);

 protected:
  /** Webview request processor manager. */
  WebUrlManager *webview_url_manager;
  /** Webview navigation manager. */
  WebNavManager *webview_nav_manager;
  /** Webview request manager. */
  WebRequestManager *webview_request_manager;
  /** Webview REST API manager. */
  WebviewRestApiManager *webview_rest_api_manager;
};

} // end namespace fawkes

#endif
