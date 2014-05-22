
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
 * @param htdocs_dirs directories in the file system where to look for static files
 * @param logger logger
 */
WebviewStaticRequestProcessor::WebviewStaticRequestProcessor(const char *baseurl,
							     std::vector<const char *> htdocs_dirs,
							     fawkes::Logger *logger)
{
  __logger         = logger;
  //store all htdocs_dirs
  if(htdocs_dirs.size() <= 0)
  {
    throw Exception(errno, "htdocs_dirs is empty");
  }
  __htdocs_dirs = std::vector<char *>(htdocs_dirs.size());
  __htdocs_dirs_len = std::vector<size_t>(htdocs_dirs.size());
  for(unsigned int i = 0; i < htdocs_dirs.size(); i++)
  {
    char htdocs_rp[PATH_MAX];
    if (realpath(htdocs_dirs[i], htdocs_rp) != NULL)
    {
      __htdocs_dirs[i]     = strdup(htdocs_rp);
      __htdocs_dirs_len[i] = strlen(__htdocs_dirs[i]);
    } else
    {
      throw Exception(errno, "Failed to resolve htdocs path '%s'", htdocs_dirs[i]);
    }
  }
  
  __baseurl        = strdup(baseurl);
  __baseurl_len    = strlen(__baseurl);
}

/** Destructor. */
WebviewStaticRequestProcessor::~WebviewStaticRequestProcessor()
{
  free(__baseurl);
  for(unsigned int i = 0; i < __htdocs_dirs.size(); i++)
  {
    free(__htdocs_dirs[i]);
  }
}


WebReply *
WebviewStaticRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if ( strncmp(__baseurl, request->url().c_str(), __baseurl_len) == 0 ) {
    // It is in our URL prefix range

    // Try all htdocs_dirs
    for(unsigned int i = 0; i < __htdocs_dirs.size(); i++)
    {
      std::string file_path = std::string(__htdocs_dirs[i]) + request->url().substr(__baseurl_len);
      
      char rf[PATH_MAX];
      char *realfile = realpath(file_path.c_str(), rf);
    
      if(realfile)
      {
	if (strncmp(realfile, __htdocs_dirs[i], __htdocs_dirs_len[i]) == 0) {
	  try {
	    DynamicFileWebReply *freply = new DynamicFileWebReply(file_path.c_str());
	    return freply;
	  } catch (fawkes::Exception &e) {
	    __logger->log_error("WebStaticReqProc",
				"Cannot fulfill request for file %s,"
				" exception follows", request->url().c_str());
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
    }
    //file not found or access forbidden
    if (errno == ENOENT) {
      return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "File not found");
    } else if (errno == EACCES) {
      return new WebErrorPageReply(WebReply::HTTP_FORBIDDEN, "Access forbidden");
    } else {
      char tmp[1024];
      if (strerror_r(errno, tmp, sizeof(tmp)) == 0) {
	return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
				     "File access failed: %s",  tmp);
      } else {
	return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
				     "File access failed: Unknown error");
      }
    }
  } else {
    // wrong base url, why the heck are we called!?
    __logger->log_error("WebStaticReqProc", "Called for invalid base url "
			"(url: %s, baseurl: %s)", request->url().c_str(), __baseurl);
    return NULL;
  }
}
