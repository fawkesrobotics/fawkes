
/***************************************************************************
 *  redirect_reply.h - Web request reply for a redirect
 *
 *  Created: Thu Feb 12 13:39:04 2009
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

#ifndef _LIBS_WEBVIEW_REDIRECT_REPLY_H_
#define _LIBS_WEBVIEW_REDIRECT_REPLY_H_

#include <webview/reply.h>

namespace fawkes {

class WebRedirectReply : public StaticWebReply
{
public:
	WebRedirectReply(std::string url);
};

} // end namespace fawkes

#endif
