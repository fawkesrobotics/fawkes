
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
#include <webview/url_manager.h>

#include <core/exception.h>
#include <logging/logger.h>

#include <cstring>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <cerrno>
#include <climits>
#include <functional>

using namespace fawkes;

/** @class WebviewStaticRequestProcessor "static_processor.h"
 * Static file web processor.
 * This processor provides access to static files.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param url_manager URL manager to register with
 * @param htdocs_dirs directories in the file system where to look for static files
 * @param logger logger
 */
WebviewStaticRequestProcessor::WebviewStaticRequestProcessor(fawkes::WebUrlManager *url_manager,
                                                             std::vector<const char *> htdocs_dirs,
                                                             fawkes::Logger *logger)
{
  logger_         = logger;
  url_manager_    = url_manager;
  //store all htdocs_dirs
  if(htdocs_dirs.size() <= 0)
  {
    throw Exception(errno, "htdocs_dirs is empty");
  }
  htdocs_dirs_ = std::vector<char *>(htdocs_dirs.size());
  htdocs_dirs_len_ = std::vector<size_t>(htdocs_dirs.size());
  for(unsigned int i = 0; i < htdocs_dirs.size(); i++)
  {
    char htdocs_rp[PATH_MAX];
    if (realpath(htdocs_dirs[i], htdocs_rp) != NULL)
    {
      htdocs_dirs_[i]     = strdup(htdocs_rp);
      htdocs_dirs_len_[i] = strlen(htdocs_dirs_[i]);
    } else
    {
      throw Exception(errno, "Failed to resolve htdocs path '%s'", htdocs_dirs[i]);
    }
  }

  url_manager_->add_handler(WebRequest::METHOD_GET, "/static/{file+}",
                            std::bind(&WebviewStaticRequestProcessor::process_request, this,
                                      std::placeholders::_1));
}

/** Destructor. */
WebviewStaticRequestProcessor::~WebviewStaticRequestProcessor()
{
	url_manager_->remove_handler(WebRequest::METHOD_GET, "/static/{file+}");
  for(unsigned int i = 0; i < htdocs_dirs_.size(); i++)
  {
    free(htdocs_dirs_[i]);
  }
}


WebReply *
WebviewStaticRequestProcessor::process_request(const fawkes::WebRequest *request)
{
	std::string filename = "/" + request->path_arg("file");

	// Try all htdocs_dirs
	for(unsigned int i = 0; i < htdocs_dirs_.size(); i++)
	{
		std::string file_path = std::string(htdocs_dirs_[i]) + filename;
      
		char rf[PATH_MAX];
		char *realfile = realpath(file_path.c_str(), rf);
    
		if(realfile)
		{
			if (strncmp(realfile, htdocs_dirs_[i], htdocs_dirs_len_[i]) == 0) {
				try {
					DynamicFileWebReply *freply = new DynamicFileWebReply(file_path.c_str());
					return freply;
				} catch (fawkes::Exception &e) {
					logger_->log_error("WebStaticReqProc",
					                   "Cannot fulfill request for file %s,"
					                   " exception follows", request->url().c_str());
					logger_->log_error("WebStaticReqProc", e);
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
}
