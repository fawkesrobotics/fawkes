
/***************************************************************************
 *  redirect_reply.h - Web request reply for a redirect
 *
 *  Created: Thu Feb 12 13:40:12 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <webview/redirect_reply.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebRedirectReply <webview/redirect_reply.h>
 * Redirect reply for webview.
 * This reply will cause an immediate redirect from the requested page
 * to the given URL. THe URL can be local as well as remote. The redirect
 * is done on the HTTP level with status code "moved permanently" and
 * the new URL as "Location" HTTP header.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param url the URL to redirect to
 */
WebRedirectReply::WebRedirectReply(std::string url)
  : StaticWebReply(WebReply::HTTP_MOVED_PERMANENTLY)
{
  add_header("Location", url);
}

} // end namespace fawkes
