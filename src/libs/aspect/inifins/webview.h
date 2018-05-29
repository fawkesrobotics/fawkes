
/***************************************************************************
 *  webview.h - Fawkes WebviewAspect initializer/finalizer
 *
 *  Created: Thu Nov 25 23:11:24 2010
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

#ifndef __ASPECT_INIFINS_WEBVIEW_H_
#define __ASPECT_INIFINS_WEBVIEW_H_

#include <aspect/inifins/inifin.h>
#include <aspect/webview.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebviewAspectIniFin : public AspectIniFin
{
 public:
  WebviewAspectIniFin();
  ~WebviewAspectIniFin();

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);

 private:
  WebUrlManager *__url_manager;
  WebNavManager *__nav_manager;
  WebRequestManager *__request_manager;
  WebviewRestApiManager *__rest_api_manager;
};

} // end namespace fawkes

#endif
