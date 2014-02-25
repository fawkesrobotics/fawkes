
/***************************************************************************
 *  error_reply.h - Web request reply for an error page
 *
 *  Created: Fri Oct 24 19:55:26 2008
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

#ifndef __LIBS_WEBVIEW_ERROR_REPLY_H_
#define __LIBS_WEBVIEW_ERROR_REPLY_H_

#include <webview/page_reply.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebErrorPageReply : public WebPageReply
{
 public:
  WebErrorPageReply(Code error_code, const char *format = NULL, ...);
};

} // end namespace fawkes

#endif
