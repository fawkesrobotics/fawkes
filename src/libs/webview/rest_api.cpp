
/***************************************************************************
 *  rest_api.cpp - Webview REST API
 *
 *  Created: Fri Mar 16 17:39:57 2018
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

#include <webview/rest_api.h>
#include <webview/router.h>

#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebviewRestApi <webview/rest_api.h>
 * Webview REST API component.
 * This class represents a specific REST API available through Webview.
 * The API's name will be part of the URL, e.g., '/api/[COMPONENT-NAME]/...'.
 * The REST API can process patterns according to the OpenAPI 3 specification.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param name of the API.
 * The API's name will be part of the URL, e.g., '/api/[COMPONENT-NAME]/...'.
 * @param logger logger for informative output
 */
WebviewRestApi::WebviewRestApi(const std::string &name, fawkes::Logger *logger)
	: name_(name), logger_(logger), pretty_json_(false), router_{std::make_shared<WebviewRouter<Handler>>()}
{
}

/** Get name of component.
 * @return name of component.
 */
const std::string &
WebviewRestApi::name() const
{
	return name_;
}

/** Process REST API request.
 * @param request incoming request
 * @param rest_url the URL stripped of the base URL prefix
 * @return reply
 */
WebReply *
WebviewRestApi::process_request(const WebRequest *request, const std::string & rest_url)
{
	try {
		std::map<std::string, std::string> path_args;
		Handler handler = router_->find_handler(request->method(), rest_url, path_args);
		WebviewRestParams params;
		params.set_path_args(std::move(path_args));
		params.set_query_args(request->get_values());
		std::unique_ptr<WebReply> reply = handler(request->body(), params);
		return reply.release();
	} catch (NullPointerException &e) {
		return NULL;
	}
}


/** Add handler function.
 * @param method HTTP method to react to
 * @param path path (after component base path) to react to
 * @param handler handler function
 */
void
WebviewRestApi::add_handler(WebRequest::Method method, std::string path, Handler handler)
{
	router_->add(method, path, handler);
}

/** Enable or disable pretty JSON printing globally.
 * @param pretty true to enable
 */
void
WebviewRestApi::set_pretty_json(bool pretty)
{
	pretty_json_ = pretty;
}
	
} // end of namespace fawkes
