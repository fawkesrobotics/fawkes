
/***************************************************************************
 *  user_verifier.h - Web request user verifier
 *
 *  Created: Mon Jan 24 18:05:47 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef _LIBS_WEBVIEW_USER_VERIFIER_H_
#define _LIBS_WEBVIEW_USER_VERIFIER_H_

namespace fawkes {

class WebUserVerifier
{
public:
	virtual ~WebUserVerifier();

	virtual bool verify_user(const char *user, const char *password) noexcept = 0;
};

} // end namespace fawkes

#endif
