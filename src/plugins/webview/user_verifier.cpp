
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
#include <utils/logging/logger.h>

#include <string>
#ifdef __USE_GNU
#  include <crypt.h>
#else
#  include <unistd.h>
#endif

#define HASH_REGEX "^(\\$([[:alnum:]]+)\\$([a-zA-Z0-9/.]+)\\$)[a-zA-Z0-9/.]+$"

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
  : config(config), logger(logger)
{
  int regerr;
  if ((regerr = regcomp(&__hash_regex, HASH_REGEX, REG_EXTENDED)) != 0) {
    char errtmp[1024];
    regerror(regerr, &__hash_regex, errtmp, sizeof(errtmp));
    regfree(&__hash_regex);
    throw Exception("Failed to compile hash regex: %s", errtmp);
  }
}


/** Destructor. */
WebviewUserVerifier::~WebviewUserVerifier()
{
  regfree(&__hash_regex);
}


bool
WebviewUserVerifier::verify_user(const char *user, const char *password) throw()
{
  try {
    std::string userpath = std::string("/webview/users/") + user;
    std::string confpass = config->get_string(userpath.c_str());

    regmatch_t m[4];
    if (regexec(&__hash_regex, confpass.c_str(), 4, m, 0) == REG_NOMATCH) {
      // assume clear text
      //logger->log_warn("WebviewUserVerifier", "Access denied for user %s, "
      //		       "invalid clear text password", user);
      return (confpass == password);
    }

#ifdef __USE_GNU
    struct crypt_data cd;
    cd.initialized = 0;

    char *crypted = crypt_r(password, confpass.c_str(), &cd);
#else
    char *crypted = crypt(password, confpass.c_str());
#endif

    if (confpass == crypted) {
      return true;
    } else {
      //logger->log_warn("WebviewUserVerifier", "Access denied for user %s, "
      //		       "invalid clear hashed password", user);
      return false;
    }

  } catch (Exception &e) {
    //logger->log_warn("WebviewUserVerifier", "Access denied for unknown user %s",
    //		     user);
    return false;
  }

  // should not actually happen, just in case...
  return false;
}
