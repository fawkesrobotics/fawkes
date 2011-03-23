
/***************************************************************************
 *  user_verifier.cpp - Web request user verifier
 *
 *  Created: Mon Jan 24 18:09:23 2011
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

#include <webview/user_verifier.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebUserVerifier <webview/user_verifier.h>
 * Interface for user verification.
 * Implementations of this class will verify users for access to the webserver.
 * Note that the password might be a hash for digest authentication.
 * @author Tim Niemueller
 *
 * @fn bool WebUserVerifier::verify_user(const char *user, const char *password) throw()
 * Verify a user.
 * Check if the passed credentials are valid.
 * @param user user name
 * @param password user supplied password, might be a hash.
 * @return true, if the credentials are valid and the user should be allowed
 * access, false otherwise.
 */

/** Virtual empty destructor. */
WebUserVerifier::~WebUserVerifier()
{
}


} // end namespace fawkes
