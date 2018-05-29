
/***************************************************************************
 *  static_processor.cpp - Web request processor for static files
 *
 *  Created: Mon Oct 13 23:41:24 2008
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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
#include <core/exceptions/system.h>
#include <core/exceptions/software.h>
#include <logging/logger.h>

#include <cstring>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <cerrno>
#include <climits>
#include <functional>
#include <regex>

#include <boost/filesystem.hpp>

using namespace fawkes;

/** @class WebviewStaticRequestProcessor "static_processor.h"
 * Static file web processor.
 * This processor provides access to static files.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param url_manager URL manager to register with
 * @param base_url base URL for static files
 * @param htdocs_dirs directories in the file system where to look for static files
 * @param catchall_file file to be served if a non-existent path is requested.
 * @param mime_file file with MIME types to read
 * @param logger logger
 */
WebviewStaticRequestProcessor::WebviewStaticRequestProcessor(fawkes::WebUrlManager *url_manager,
                                                             const std::string& base_url,
                                                             std::vector<std::string>& htdocs_dirs,
                                                             const std::string& catchall_file,
                                                             const std::string& mime_file,
                                                             fawkes::Logger *logger)
{
  logger_         = logger;
  url_manager_    = url_manager;
  base_url_       = base_url;

  if(htdocs_dirs.size() <= 0) {
    throw Exception(errno, "htdocs_dirs is empty");
  }
  for(const auto &h : htdocs_dirs) {
    char htdocs_rp[PATH_MAX];
    if (realpath(h.c_str(), htdocs_rp) != NULL) {
	    htdocs_dirs_.push_back(htdocs_rp);
    } else {
	    throw Exception(errno, "Failed to resolve htdocs path '%s'", h.c_str());
    }
  }

  catchall_file_  = catchall_file;
  //logger_->log_debug("WebStaticReqProc", "Catch-all file: %s", catchall_file_.c_str());

  read_mime_database(mime_file);
  
  url_manager_->add_handler(WebRequest::METHOD_GET, base_url + "{file+}",
                            std::bind(&WebviewStaticRequestProcessor::process_request, this,
                                      std::placeholders::_1), 10040);

  if (catchall_file_ != "") {
	  url_manager_->add_handler(WebRequest::METHOD_GET, base_url + "?",
	                            std::bind(&WebviewStaticRequestProcessor::process_request, this,
	                                      std::placeholders::_1), 10050);
  }
}

/** Destructor. */
WebviewStaticRequestProcessor::~WebviewStaticRequestProcessor()
{
	url_manager_->remove_handler(WebRequest::METHOD_GET, base_url_ + "{file+}");
  if (catchall_file_ != "") {
	  url_manager_->remove_handler(WebRequest::METHOD_GET, base_url_ + "?");
  }
}

void
WebviewStaticRequestProcessor::read_mime_database(const std::string& mime_file)
{
	std::regex words_regex("[^\\s]+");

	mime_types_["unknown"] = "";
	
	std::ifstream f(mime_file);
	for (std::string line; std::getline(f, line); ) {
		if (line[0] == '#')  continue;

		auto words_begin = std::sregex_iterator(line.begin(), line.end(), words_regex);
		auto words_end = std::sregex_iterator();
		if (words_begin == words_end) continue;

		std::string mime_type = words_begin->str();
		for (std::sregex_iterator i = ++words_begin; i != words_end; ++i) {
			mime_types_[i->str()] = mime_type;
		}
	}
	logger_->log_debug("WebStaticReqProc", "Read %zu mime types from '%s'",
	                   mime_types_.size(), mime_file.c_str());
}


const std::string &
WebviewStaticRequestProcessor::get_mime_type(const std::string& file_name)
{
	std::string::size_type dot_pos = file_name.rfind(".");
	if (dot_pos == std::string::npos) {
		return mime_types_["unknown"];
	}
	const auto &m = mime_types_.find(file_name.substr(dot_pos+1));
	if (m != mime_types_.end()) {
		return m->second;
	} else {
		return mime_types_["unknown"];
	}		
}

std::string
WebviewStaticRequestProcessor::find_file(const std::string& filename)
{
	for(const auto &h : htdocs_dirs_) {
		std::string file_path = h + filename;
		char rf[PATH_MAX];
		char *realfile = realpath(file_path.c_str(), rf);
    
		if (realfile) {
			if (boost::filesystem::is_directory(realfile))  continue;

			if (strncmp(realfile, h.c_str(), h.length()) == 0) {
				if (access(realfile, R_OK) == 0) {
					return realfile;
				} else {
					switch (errno) {
					case EACCES:
						throw AccessViolationException("Access forbidden (file permission)");
					default:
						throw IllegalArgumentException("Failed to open %s: %s", filename.c_str(), strerror(errno));
					}
				}
			} else {
				throw AccessViolationException("Access forbidden (breakout)");
			}
		}
	}
	throw CouldNotOpenFileException(filename.c_str(), 0);
}


WebReply *
WebviewStaticRequestProcessor::process_request(const fawkes::WebRequest *request)
{
	try {
		std::string filename = find_file("/" + request->path_arg("file"));
		try {
			DynamicFileWebReply *freply = new DynamicFileWebReply(filename, get_mime_type(filename));
			return freply;
		} catch (fawkes::Exception &e) {
			logger_->log_error("WebStaticReqProc",
			                   "Cannot fulfill request for file %s: %s",
			                   request->url().c_str(), e.what_no_backtrace());
			return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
			                             e.what_no_backtrace());
		}
	} catch (AccessViolationException &e) {
		// Someone tries to trick us to give away files we don't want to give
		logger_->log_error("WebStaticReqProc", "Access denied for %s: %s",
		                   request->url().c_str(), e.what_no_backtrace());
		return new WebErrorPageReply(WebReply::HTTP_FORBIDDEN, e.what_no_backtrace());
	} catch (IllegalArgumentException &e) {
		logger_->log_error("WebStaticReqProc", "Failed to serve %s: %s",
		                   request->url().c_str(), e.what_no_backtrace());
		return new WebErrorPageReply(WebReply::HTTP_BAD_REQUEST, e.what_no_backtrace());
	} catch (CouldNotOpenFileException &e) {
		std::string catchall_file;
		try {
			catchall_file = std::move(find_file("/" + catchall_file_));
		} catch (Exception &e) {} // ignore, serve 404

		if (catchall_file.empty()) {
			if (catchall_file_.empty()) {
				return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "File not found");
			} else {
				return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "File not found. <i>Frontend not deployed?</i>");
			}
		} else {
			try {
				DynamicFileWebReply *freply = new DynamicFileWebReply(catchall_file,
				                                                      get_mime_type(catchall_file));
				return freply;
			} catch (Exception &e) {
				logger_->log_error("WebStaticReqProc", "Failed to serve catchall file: %s",
				                   e.what_no_backtrace());
				return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
				                             e.what_no_backtrace());
			}
		}
	}
}
