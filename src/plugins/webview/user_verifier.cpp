
/***************************************************************************
 *  user_verifier.cpp - Webview user verifier
 *
 *  Created: Mon Jan 24 18:43:47 2011
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

#include "user_verifier.h"

#include <core/exception.h>
#include <config/config.h>
#include <logging/logger.h>

#include <string>
#ifdef HAVE_CRYPT
#  ifdef __USE_GNU
#    include <crypt.h>
#  else
#    include <unistd.h>
#  endif
#endif
#ifdef HAVE_APR_UTIL
#  include <apr_md5.h>
#endif
using namespace fawkes;

/** @class WebviewUserVerifier "user_verifier.h"
 * Webview user verification.
 * Verifies users against entries in the configuration database.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config configuration to read users from
 * @param logger logger for log output
 */
WebviewUserVerifier::WebviewUserVerifier(Configuration *config, Logger *logger)
  : config(config)
{
}


/** Destructor. */
WebviewUserVerifier::~WebviewUserVerifier()
{
}


bool
WebviewUserVerifier::verify_user(const char *user, const char *password) throw()
{
  try {
    std::string userpath = std::string("/webview/users/") + user;
    std::string confpass = config->get_string(userpath.c_str());

    if (confpass.find("!cleartext!") == 0) {
      return (confpass.substr(11) == password);
    }

#ifdef HAVE_APR_UTIL
    return
      (apr_password_validate(password, confpass.c_str()) == APR_SUCCESS);

#elif defined(HAVE_CRYPT)
#  ifdef __USE_GNU
    struct crypt_data cd;
    cd.initialized = 0;

    char *crypted = crypt_r(password, confpass.c_str(), &cd);
#  else
    char *crypted = crypt(password, confpass.c_str());
#  endif

    if (confpass == crypted) {
      return true;
    } else {
      //logger->log_warn("WebviewUserVerifier", "Access denied for user %s, "
      //		       "invalid clear hashed password", user);
      return false;
    }
#else
    return (confpass == password);
#endif

  } catch (Exception &e) {
    //logger->log_warn("WebviewUserVerifier", "Access denied for unknown user %s",
    //		     user);
    return false;
  }

  // should not actually happen, just in case...
  return false;
}
