
/***************************************************************************
 *  static_processor.cpp - Web request processor for static files
 *
 *  Created: Mon Oct 13 23:41:24 2008
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

#include "static_processor.h"
#include <webview/file_reply.h>
#include <webview/error_reply.h>

#include <core/exception.h>
#include <logging/logger.h>

#include <cstring>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <cerrno>
#include <climits>

using namespace fawkes;

/** @class WebviewStaticRequestProcessor "static_processor.h"
 * Static file web processor.
 * This processor provides access to static files.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param baseurl Base URL where the static processor is mounted
 * @param htdocs_dir directory in the file system where to look for static files
 * @param logger logger
 */
WebviewStaticRequestProcessor::WebviewStaticRequestProcessor(const char *baseurl,
						     const char *htdocs_dir,
						     fawkes::Logger *logger)
{
  __logger         = logger;
  __baseurl        = strdup(baseurl);
  __baseurl_len    = strlen(__baseurl);
  __htdocs_dir     = strdup(htdocs_dir);
  __htdocs_dir_len = strlen(__htdocs_dir);

}

/** Destructor. */
WebviewStaticRequestProcessor::~WebviewStaticRequestProcessor()
{
  free(__baseurl);
  free(__htdocs_dir);
}


WebReply *
WebviewStaticRequestProcessor::process_request(const char *url,
					   const char *method,
					   const char *version,
					   const char *upload_data,
					   size_t *upload_data_size,
					   void **session_data)
{
  if ( strncmp(__baseurl, url, __baseurl_len) == 0 ) {
    // It is in our URL prefix range
    std::string file_path = std::string(__htdocs_dir) + std::string(url).substr(__baseurl_len);

    char rf[PATH_MAX];
    char *realfile = realpath(file_path.c_str(), rf);
    if (! realfile ) {
      if (errno == ENOENT) {
	return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "File not found");
      } else if (errno == EACCES) {
	return new WebErrorPageReply(WebReply::HTTP_FORBIDDEN, "Access forbidden");
      } else {
	char tmp[1024];
	strerror_r(errno, tmp, sizeof(tmp));
	return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
				     "File access failed: %s",  tmp);
      }
    } else {
      if (strncmp(realfile, __htdocs_dir, __htdocs_dir_len) == 0) {
	try {
	  DynamicFileWebReply *freply = new DynamicFileWebReply(file_path.c_str());
	  return freply;
	} catch (fawkes::Exception &e) {
	  __logger->log_error("WebStaticReqProc",
			      "Cannot fulfill request for file %s,"
			      " exception follows", url);
	  __logger->log_error("WebStaticReqProc", e);
	  return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
				       *(e.begin()));
	}
      } else {
	// Someone tries to trick us to give away files we don't want to give
	return new WebErrorPageReply(WebReply::HTTP_FORBIDDEN,
				     "Access forbidden, breakout detected.");
      }
    }
  } else {
    // wrong base url, why the heck are we called!?
    __logger->log_error("WebStaticReqProc", "Called for invalid base url "
			"(url: %s, baseurl: %s)", url, __baseurl);
    return NULL;
  }
}
