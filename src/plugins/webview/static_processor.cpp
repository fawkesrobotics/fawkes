
/***************************************************************************
 *  static_processor.cpp - Web request processor for static files
 *
 *  Created: Mon Oct 13 23:41:24 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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
#include "file_reply.h"

#include <core/exception.h>
#include <utils/logging/logger.h>

#include <cstring>
#include <cstdlib>
#include <string>
#include <unistd.h>

/** @class WebStaticRequestProcessor "static_processor.h"
 * Static file web processor.
 * This processor provides access to static files.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param baseurl Base URL where the static processor is mounted
 * @param htdocs_dir directory in the file system where to look for static files
 * @param logger logger
 */
WebStaticRequestProcessor::WebStaticRequestProcessor(const char *baseurl,
						     const char *htdocs_dir,
						     fawkes::Logger *logger)
{
  __logger      = logger;
  __baseurl     = strdup(baseurl);
  __baseurl_len = strlen(__baseurl);
  __htdocs_dir  = strdup(htdocs_dir);
}

/** Destructor. */
WebStaticRequestProcessor::~WebStaticRequestProcessor()
{
  free(__baseurl);
  free(__htdocs_dir);
}


WebReply *
WebStaticRequestProcessor::process_request(const char *url,
					   const char *method,
					   const char *version,
					   const char *upload_data,
					   unsigned int *upload_data_size,
					   void **session_data)
{
  if ( strncmp(__baseurl, url, __baseurl_len) == 0 ) {
    // It is in our URL prefix range
    std::string file_path = std::string(__htdocs_dir) + std::string(url).substr(__baseurl_len);

    try {
      DynamicFileWebReply *freply = new DynamicFileWebReply(file_path.c_str());
      return freply;
    } catch (fawkes::Exception &e) {
      __logger->log_error("WebStaticReqProc", "Cannot fulfill request for file %s,"
			  " exception follows", url);
      __logger->log_error("WebStaticReqProc", e);
      return new StaticWebReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
				*(e.begin()));
    }
  } else {
    return NULL;
  }
}
