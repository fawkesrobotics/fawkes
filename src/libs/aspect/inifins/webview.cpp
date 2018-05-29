
/***************************************************************************
 *  webview.cpp - Fawkes WebviewAspect initializer/finalizer
 *
 *  Created: Thu Nov 25 23:12:01 2010
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

#include <aspect/inifins/webview.h>
#include <aspect/webview.h>
#include <core/threading/thread_finalizer.h>
#include <webview/url_manager.h>
#include <webview/nav_manager.h>
#include <webview/request_manager.h>
#include <webview/rest_api_manager.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebviewAspectIniFin <aspect/inifins/webview.h>
 * Initializer/finalizer for the WebviewAspect.
 * @author Tim Niemueller
 */

/** Constructor. */
WebviewAspectIniFin::WebviewAspectIniFin()
  : AspectIniFin("WebviewAspect")
{
  __url_manager = new WebUrlManager();
  __nav_manager = new WebNavManager();
  __request_manager = new WebRequestManager();
  __rest_api_manager = new WebviewRestApiManager();
}

/** Destructor. */
WebviewAspectIniFin::~WebviewAspectIniFin()
{
  delete __url_manager;
  delete __nav_manager;
  delete __request_manager;
  delete __rest_api_manager;
}


void
WebviewAspectIniFin::init(Thread *thread)
{
  WebviewAspect *webview_thread;
  webview_thread = dynamic_cast<WebviewAspect *>(thread);
  if (webview_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "WebviewAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  webview_thread->init_WebviewAspect(__url_manager, __nav_manager,
                                     __request_manager, __rest_api_manager);
}


void
WebviewAspectIniFin::finalize(Thread *thread)
{
  WebviewAspect *webview_thread;
  webview_thread = dynamic_cast<WebviewAspect *>(thread);
  if (webview_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"WebviewAspect, but RTTI says it "
					"has not. ", thread->name());
  }
}

} // end namespace fawkes
