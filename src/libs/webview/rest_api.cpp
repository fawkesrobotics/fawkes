
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

std::pair<std::regex, std::vector<std::string>>
WebviewRestApi::gen_regex(const std::string &  path)
{
	if (path[0] != '/') {
		throw Exception("Path '%s' must start with /", path.c_str());
	}

	std::regex to_re("\\{([^+\\}]+?)\\+?\\}");
	std::string m_path = path;
	std::string re_url;
	std::smatch match;
	std::vector<std::string> match_indexes;
	while (regex_search(m_path, match, to_re)) {
		std::string full_match = match[0];
		re_url += match.prefix();
		if (full_match[full_match.length()-2] == '+') {
			re_url += "(.+?)";
		} else {
			re_url += "([^/]+?)";
		}
		match_indexes.push_back(match[1]);
		m_path = match.suffix();
	}
	re_url += m_path;

	return std::make_pair(std::regex(re_url), match_indexes);
}
	
/** Check if an actual path matches an API path pattern.
 * @param url requested
 * @param api_path configured API path to check
 * @param params object to set argument mappings
 * @return true if the path cold be matched, false otherwise.
 */
bool
WebviewRestApi::path_match(const std::string & url, const path_regex &path_re,
                           WebviewRestParams& params)
{
	std::map<std::string, std::string> path_args;
	std::smatch matches;
	if (std::regex_match(url, matches, path_re.first)) {
		if (matches.size() != path_re.second.size() + 1) {
			return false;
		}
		for (size_t i = 0; i < path_re.second.size(); ++i) {
			path_args[path_re.second[i]] = matches[i+1].str();
		}
		params.set_path_args(std::move(path_args));
		return true;
	} else {
		return false;
	}
}

/** Process REST API request.
 * @param request incoming request
 * @param rest_url the URL stripped of the base URL prefix
 * @return reply
 */
WebReply *
WebviewRestApi::process_request(const WebRequest *request, const std::string & rest_url)
{
	WebviewRestParams params;
	auto ri = std::find_if(routes_.begin(), routes_.end(),
	                       [this, rest_url, &params, request](auto r) {
		                       if (std::get<0>(r) != request->method()) {
			                       return false;
		                       }
		                       if (this->path_match(rest_url, std::get<2>(r), params)) {
			                       return true;
		                       } else {
			                       return false;
		                       }
	                       });
	if (ri == routes_.end()) {
		return new StaticWebReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, "API " + rest_url + " unknown");
	}
	params.set_query_args(request->get_values());
	std::unique_ptr<WebviewRestReply> reply = std::get<3>(*ri)(request->body(), params);
	return reply.release();
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
