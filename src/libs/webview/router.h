
/***************************************************************************
 *  router.h - Webview Router
 *
 *  Created: Thu Mar 29 21:45:17 2018
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


#ifndef __LIBS_WEBVIEW_ROUTER_H_
#define __LIBS_WEBVIEW_ROUTER_H_

#include <core/exceptions/software.h>

#include <string>
#include <map>
#include <algorithm>
#include <list>
#include <regex>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** URL path router.
 * Register URL path patterns and some handler or item. Then match it
 * later to request URLs to retrieve this very handler or item if the
 * pattern matches the URL.
 * @author Tim Niemueller
 */
template <typename T>
class WebviewRouter
{
 public:
	/** Find a handler.
	 * @param request incoming request object
	 * @param path_args upon successful completion, will contain mappings from
	 * path patterns to matched segments of the URL.
	 * @return matched handler
	 * @exception NullPointerException thrown if no handler could be found
	 */
	T &
	find_handler(const WebRequest *request, std::map<std::string, std::string>& path_args)
	{
		auto ri = std::find_if(routes_.begin(), routes_.end(),
		                       [this, &path_args, request](auto r) -> bool {
			                       //printf("Comparing %s to %s\n", request->url().c_str(), std::get<2>(r).c_str());
			                       return (std::get<1>(r) == request->method() &&
			                               this->path_match(request->url(), std::get<3>(r), path_args));
		                       });
		if (ri == routes_.end()) {
			throw NullPointerException("No handler found");
		}
		return std::get<4>(*ri);
	}

	/** Find a handler.
	 * @param method HTTP method of the request
	 * @param path path to match against stored paths
	 * @param path_args upon successful completion, will contain mappings from
	 * path patterns to matched segments of the URL.
	 * @return matched handler
	 * @exception NullPointerException thrown if no handler could be found
	 */
	T &
	find_handler(WebRequest::Method method, const std::string& path,
	             std::map<std::string, std::string>& path_args)
	{
		auto ri = std::find_if(routes_.begin(), routes_.end(),
		                       [this, &path_args, &method, &path](auto r) -> bool {
			                       return (std::get<1>(r) == method &&
			                               this->path_match(path, std::get<3>(r), path_args));
		                       });
		if (ri == routes_.end()) {
			throw NullPointerException("No handler found");
		}
		return std::get<4>(*ri);
	}

	/** Add a handler with weight.
	 * @param method HTTP method to match for
	 * @param path path pattern. A pattern may contain "{var}" segments
	 * for a URL. These will match an element of the path, i.e., a string not
	 * containing a slash. If a pattern has the form {var+} then it may contain
	 * a slash and therefore match multiple path segments. The handler would
	 * receive an entry named "var" in the parameters path arguments.
	 * @param handler handler to store
	 * @param weight higher weight means the handler is tried later by
	 * the router. The default is 0.
	 */
	void
	add(WebRequest::Method method, const std::string &path, T handler, int weight)
	{
		auto ri = std::find_if(routes_.begin(), routes_.end(),
		                       [this, method, &path, &weight](auto r) -> bool {
			                       return (std::get<0>(r) == weight &&
			                               std::get<1>(r) == method &&
			                               std::get<2>(r) == path);
		                       });
		if (ri != routes_.end()) {
			throw Exception("URL handler already registered for %s", path.c_str());
		}
		routes_.push_back(std::make_tuple(weight, method, path, gen_regex(path), handler));
		routes_.sort([](const auto &a, const auto &b) -> bool
		             { return (std::get<0>(a) < std::get<0>(b)); });
	}

	/** Add a handler.
	 * @param method HTTP method to match for
	 * @param path path pattern. A pattern may contain "{var}" segments
	 * for a URL. These will match an element of the path, i.e., a string not
	 * containing a slash. If a pattern has the form {var+} then it may contain
	 * a slash and therefore match multiple path segments. The handler would
	 * receive an entry named "var" in the parameters path arguments.
	 * @param handler handler to store
	 */
	void
	add(WebRequest::Method method, const std::string &path, T handler)
	{
		add(method, path, handler, 0);
	}
	
	/** Remove a handler.
	 * @param method HTTP method to match for
	 * @param path path pattern that equals the one given when adding.
	 */
	void
	remove(WebRequest::Method method, const std::string &path)
	{
		auto ri = std::find_if(routes_.begin(), routes_.end(),
		                       [this, method, &path](auto r) -> bool {
			                       return (std::get<1>(r) == method &&
			                               std::get<2>(r) == path);
		                       });
		if (ri != routes_.end()) {
			routes_.erase(ri);
		}
	}

 private:
	typedef std::pair<std::regex, std::vector<std::string>> path_regex;

	std::pair<std::regex, std::vector<std::string>>
		gen_regex(const std::string &  path)
	{
		std::string::size_type pos = 0;

		if (path[0] != '/') {
			throw Exception("Path '%s' must start with /", path.c_str());
		}
		if ((pos = path.find_first_of("[]()^$")) != std::string::npos) {
			throw Exception("Found illegal character '%c' at position '%zu' in '%s'",
			                path[pos], pos, path.c_str());
		}

		std::regex to_re("\\{([^+*\\}]+?)[+*]?\\}");
		std::string m_path = path;
		// escape special characters for regex
		pos = 0;
		while ((pos = m_path.find_first_of(".", pos)) != std::string::npos) {
			m_path.replace(pos, 1, "\\.");
			pos += 2;
		}
		pos = 0;
		while ((pos = m_path.find_first_of("+*", pos)) != std::string::npos) {
			if (pos < m_path.length() - 1 && m_path[pos+1] != '}') {
				m_path.replace(pos, 1, std::string("\\")+m_path[pos]);
				pos += 2;
			} else {
				pos += 1;
			}
		}
		std::string re_url;
		std::smatch match;
		std::vector<std::string> match_indexes;
		while (regex_search(m_path, match, to_re)) {
			std::string full_match = match[0];
			re_url += match.prefix();
			if (full_match[full_match.length()-2] == '+') {
				re_url += "(.+?)";
			} else if (full_match[full_match.length()-2] == '*') {
				re_url += "(.*)";
			} else {
				re_url += "([^/]+?)";
			}
			match_indexes.push_back(match[1]);
			m_path = match.suffix();
		}
		re_url += m_path;
		//printf("Regex: %s -> %s\n", path.c_str(), re_url.c_str());

		return std::make_pair(std::regex(re_url), match_indexes);
	}
	
	/** Check if an actual path matches an API path pattern.
	 * @param url requested
	 * @param api_path configured API path to check
	 * @param params object to set argument mappings
	 * @return true if the path cold be matched, false otherwise.
	 */
	bool path_match(const std::string & url, const path_regex &path_re,
	                std::map<std::string, std::string>& path_args)
	{
		std::smatch matches;
		if (std::regex_match(url, matches, path_re.first)) {
			if (matches.size() != path_re.second.size() + 1) {
				return false;
			}
			for (size_t i = 0; i < path_re.second.size(); ++i) {
				//printf("arg %s = %s\n", path_re.second[i].c_str(), matches[i+1].str().c_str());
				path_args[path_re.second[i]] = matches[i+1].str();
			}
			return true;
		} else {
			return false;
		}	
	}

 private:
	std::list<std::tuple<int, WebRequest::Method, std::string, path_regex, T>> routes_;
};

} // end of namespace fawkes

#endif
