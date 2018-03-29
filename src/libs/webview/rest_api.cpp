
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

#include "rest_api.h"

#include <core/exception.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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
		handler_func handler = router_.find_handler(request->method(), rest_url, path_args);
		WebviewRestParams params;
		params.set_path_args(std::move(path_args));
		params.set_query_args(request->get_values());
		std::unique_ptr<WebviewRestReply> reply = handler(request->body(), params);
		return reply.release();
	} catch (NullPointerException &e) {
		return new StaticWebReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, "API " + rest_url + " unknown");
	}
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
