
/***************************************************************************
 *  rest_processor.cpp - Web request processor for an extensible REST API
 *
 *  Created: Fri Mar 16 12:02:53 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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
#include <webview/url_manager.h>

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
 * @param url_manager URL manager to register with
 * @param api_mgr REST API manager to check for available APIs
 * @param logger logger
 */
WebviewRESTRequestProcessor::WebviewRESTRequestProcessor(fawkes::WebUrlManager *url_manager,
                                                         fawkes::WebviewRestApiManager *api_mgr,
                                                         fawkes::Logger *logger)
{
  logger_  = logger;
  api_mgr_ = api_mgr;
  url_mgr_ = url_manager;

  methods_ = {
	  WebRequest::METHOD_GET, WebRequest::METHOD_POST,
	  WebRequest::METHOD_PUT, WebRequest::METHOD_DELETE,
	  WebRequest::METHOD_PATCH};

  for (const auto &method : methods_) {
	  url_mgr_->add_handler(method, "/api/{rest_url*}",
	                        std::bind(&WebviewRESTRequestProcessor::process_request, this,
	                                  std::placeholders::_1));
  }
}

/** Destructor. */
WebviewRESTRequestProcessor::~WebviewRESTRequestProcessor()
{
  for (const auto &method : methods_) {
	  url_mgr_->remove_handler(method, "/api/{rest_url*}");
  }
}

WebReply *
WebviewRESTRequestProcessor::process_request(const fawkes::WebRequest *request)
{
	std::string rest_url = "/" + request->path_arg("rest_url");
	std::vector<std::string> rest_url_parts{str_split(rest_url, '/')};

	if (rest_url_parts.empty()) {
		return new StaticWebReply(WebReply::HTTP_NOT_FOUND, "REST API overview not yet implemented\n");
	}

	std::string rest_path = rest_url.substr(rest_url_parts[0].length()+1);
	std::string rest_api = rest_url_parts[0];
	WebviewRestApi *api = api_mgr_->get_api(rest_api);
	if (! api) {
		logger_->log_error("WebRESTProc", "REST API '%s' unknown", rest_api.c_str());
		return new StaticWebReply(WebReply::HTTP_NOT_FOUND,
		                           "REST API '" + rest_api + "' unknown\n");
	}

	try {
		WebReply *reply = api->process_request(request, rest_path);
		if (! reply) {
			return new StaticWebReply(WebReply::HTTP_NOT_FOUND, "REST API '" + rest_api +
			                          "' has no endpoint '" + rest_path + "'\n");
		}
		return no_caching(reply);
	} catch (Exception &e) {
		logger_->log_error("WebRESTProc", "REST API '%s' failed, exception follows", rest_api.c_str());
		logger_->log_error("WebRESTProc", e);
		return no_caching(new StaticWebReply(WebReply::HTTP_INTERNAL_SERVER_ERROR,
		                                     "REST API '" + rest_api + "': " +
		                                     e.what_no_backtrace() + "\n"));
	}
	
}
