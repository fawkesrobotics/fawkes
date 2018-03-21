
/***************************************************************************
 *  rest_processor.cpp - Web request processor for an extensible REST API
 *
 *  Created: Fri Mar 16 12:02:53 2018
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

#include "rest_processor.h"
#include <webview/error_reply.h>
#include <webview/rest_api_manager.h>
#include <webview/rest_api.h>

#include <core/exception.h>
#include <logging/logger.h>
#include <utils/misc/string_split.h>

#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

using namespace fawkes;

/** @class WebviewRESTRequestProcessor "rest_processor.h"
 * REST API web processor.
 * This processor provides an extensible REST API.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param baseurl Base URL where the static processor is mounted
 * @param api_mgr REST API manager to check for available APIs
 * @param logger logger
 */
WebviewRESTRequestProcessor::WebviewRESTRequestProcessor(const char *baseurl,
                                                         fawkes::WebviewRestApiManager *api_mgr,
                                                         fawkes::Logger *logger)
{
  logger_  = logger;
  baseurl_ = baseurl;
  api_mgr_ = api_mgr;
}

/** Destructor. */
WebviewRESTRequestProcessor::~WebviewRESTRequestProcessor()
{
}


WebReply *
WebviewRESTRequestProcessor::process_request(const fawkes::WebRequest *request)
{
	if (request->url().find(baseurl_) != 0) {
		logger_->log_error("WebRESTProc", "Called for invalid base url "
		                   "(url: %s, baseurl: %s)", request->url().c_str(), baseurl_.c_str());
		return NULL;
	}

	std::string rest_url = request->url().substr(baseurl_.length() + 1);
	std::vector<std::string> rest_url_parts{str_split(rest_url, '/')};

	if (rest_url_parts.empty()) {
		// return list of apis
		return NULL;
	}

	std::string rest_path = rest_url.substr(rest_url_parts[0].length() + 1);

	std::string rest_api = rest_url_parts[0];
	WebviewRestApi *api = api_mgr_->get_api(rest_api);
	if (! api) {
		logger_->log_error("WebRESTProc", "REST API '%s' unknown", rest_api.c_str());
		return new WebErrorPageReply(WebReply::HTTP_NOT_FOUND, "REST API '%s' unknown", rest_api.c_str());
	}

	try {
		WebReply *reply = api->process_request(request, rest_path);
		reply->add_header("Access-Control-Allow-Origin", "*");
		return reply;
	} catch (Exception &e) {
		logger_->log_error("WebRESTProc", "REST API '%s' failed, exception follosws", rest_api.c_str());
		logger_->log_error("WebRESTProc", e);
		return new WebErrorPageReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, "REST API '%s': %s",
		                             rest_api.c_str(), e.what_no_backtrace());
	}
	
}
