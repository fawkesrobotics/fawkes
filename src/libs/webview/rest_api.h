
/***************************************************************************
 *  reply.h - Web request reply
 *
 *  Created: Fri Mar 16 17:39:57 2018
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

#ifndef __LIBS_WEBVIEW_REST_API_H_
#define __LIBS_WEBVIEW_REST_API_H_


#include <webview/request.h>
#include <webview/reply.h>

#include <core/exception.h>
#include <logging/logger.h>
#include <utils/misc/string_split.h>

#include <cstdio>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <functional>
#include <algorithm>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** REST reply via Webview.
 * @author Tim Niemueller
 */
class WebviewRestReply : public StaticWebReply
{
 public:
	/** Constructor.
	 * @param code HTTP response code
	 * @param body body of reply, usually a JSON document.
	 */
	WebviewRestReply(WebReply::Code code, std::string body = "")
		: StaticWebReply(code, body)
	{}
};

/** REST processing exception.
 * Use to indicate failure with more specific response. The HTTP code
 * will be used for the static response with the formatted error message.
 * @author Tim Niemueller
 */
class WebviewRestException : public fawkes::Exception
{
 public:
	/** Constructor.
	 * @param code HTTP response code
	 * @param format format string for error message (cf. printf)
	 */
	WebviewRestException(WebReply::Code code, const char *format, ...)
		: Exception(), code_(code)
	{
		va_list va;
		va_start(va, format);
		append_va(format, va);
		va_end(va);
	}

	/** Get HTTP response code.
	 * @return HTTP response code
	 */
	WebReply::Code code()
	{
		return code_;
	}
	
 private:
	WebReply::Code code_;
};

/** REST parameters to pass to handlers.
 * @author Tim Niemueller
 */
class WebviewRestParams
{
	/// REST API can call private methods.
	friend class WebviewRestApi;
 public:
	/** Constructor. */
	WebviewRestParams()
		: pretty_json_(false) {}

	/** Get a path argument.
	 * Retrieves a named argument that was a token in the
	 * registration URL, e.g., retrieve "id" for "/item/{id}".
	 * @param what what to retrieve
	 * @return item passed in URL or empty string
	 */
	std::string path_arg(const std::string& what)
	{
		if (path_args_.find(what) != path_args_.end()) {
			return path_args_[what];
		} else {
			return "";
		}
	}

	/** Get a query argument.
	 * Retrieves a named query argument that was passed in the
	 * URL, e.g., retrieve "pretty" for "?pretty=true".
	 * @param what what to retrieve
	 * @return item passed in URL or empty string
	 */
	std::string query_arg(const std::string& what)
	{
		if (query_args_.find(what) != query_args_.end()) {
			return query_args_[what];
		} else {
			return "";
		}
	}

	/** Is pretty-printed JSON enabled?
	 * @return true true to request enabling pretty mode
	 */
	bool pretty_json()
	{
		return pretty_json_;
	}

	/** Enable or disable pretty printed results.
	 * Note that this only works when using the generated API
	 * interface and classes which support the "pretty" flag.
	 * @param pretty true to enable, false to disable
	 */
	void set_pretty_json(bool pretty)
	{
		pretty_json_ = pretty;
	}

 private:
	void set_path_args(std::map<std::string, std::string>&& args)
	{
		path_args_ = std::move(args);
	}

	void set_query_args(const std::map<std::string, std::string>& args)
	{
		query_args_ = args;
	}

 private:
	bool pretty_json_;
	std::map<std::string, std::string> path_args_;
	std::map<std::string, std::string> query_args_;
};

/** REST API call handler function type. */
typedef std::function<std::unique_ptr<WebviewRestReply> (std::string, WebviewRestParams&)> handler_func;

class Logger;

/** Webview REST API component.
 * This class represents a specific REST API available through Webview.
 * The API's name will be part of the URL, e.g., '/api/[COMPONENT-NAME]/...'.
 * The REST API can process patterns according to the OpenAPI 3 specification.
 * @author Tim Niemueller
 */
class WebviewRestApi
{
 public:
	/** Constructor.
	 * @param name of the API.
	 * The API's name will be part of the URL, e.g., '/api/[COMPONENT-NAME]/...'.
	 * @param logger logger for informative output
	 */
 WebviewRestApi(const std::string &name, fawkes::Logger *logger)
	 : name_(name), logger_(logger), pretty_json_(false) {}

	/** Get name of component.
	 * @return name of component.
	 */
	const std::string &
	name() const
	{
		return name_;
	}

	/** Add handler function.
	 * @param method HTTP method to react to
	 * @param path path (after component base path) to react to
	 * @param handler handler function
	 */
	void add_handler(WebRequest::Method method, std::string path, handler_func handler)
	{
		routes_.push_back(std::make_tuple(method, path, handler));
	}

	/** Add handler function.
	 * @param method HTTP method to react to
	 * @param path path (after component base path) to react to
	 * @param handler handler function
	 */
	template <class I, class O>
	void
	add_handler(WebRequest::Method method, std::string path,
		            std::function<O (I, WebviewRestParams &)> handler)
	{
		routes_.push_back(std::make_tuple(method, path,
		                                  [this, handler](const std::string &body, WebviewRestParams& m)
		                                  -> std::unique_ptr<WebviewRestReply>
		                                  {
			                                  I input;
			                                  input.from_json(body);
			                                  try {
				                                  O output{handler(input, m)};
				                                  try {
					                                  output.validate();
				                                  } catch (std::runtime_error &e) {
					                                  logger_->log_warn(("RestAPI|" + name_).c_str(), "%s", e.what());
				                                  }
				                                  return std::make_unique<WebviewRestReply>
					                                  (WebReply::HTTP_OK, output.to_json(pretty_json_ || m.pretty_json()));
			                                  } catch (WebviewRestException &e) {
				                                  return std::make_unique<WebviewRestReply>
					                                  (e.code(), e.what_no_backtrace());
			                                  } catch (Exception &e) {
				                                  auto r = std::make_unique<WebviewRestReply>
					                                  (WebReply::HTTP_INTERNAL_SERVER_ERROR);
				                                  r->append_body("Execution failed: %s", e.what_no_backtrace());
				                                  return r;
			                                  }
		                                   }));
	}

	/** Add handler function.
	 * @param method HTTP method to react to
	 * @param path path (after component base path) to react to
	 * @param handler handler function
	 */
	template <class I>
	void add_handler(WebRequest::Method method, std::string path,
	                 std::function<std::string (I, WebviewRestParams &)> handler)
	{
		routes_.push_back(std::make_tuple(method, path,
		                                  [this, handler](const std::string &body, WebviewRestParams& m)
		                                  -> std::unique_ptr<WebviewRestReply>
		                                  {
			                                  I input;
			                                  input.from_json(body);
			                                  try {
				                                  return std::make_unique<WebviewRestReply>
					                                  (WebReply::HTTP_OK, handler(input, m));
			                                  } catch (WebviewRestException &e) {
				                                  return std::make_unique<WebviewRestReply>
					                                  (e.code(), e.what_no_backtrace());
			                                  } catch (Exception &e) {
				                                  return std::make_unique<WebviewRestReply>
					                                  (WebReply::HTTP_INTERNAL_SERVER_ERROR,
					                                   "Execution failed: %s", e.what_no_backtrace());
			                                  }
		                                  }));
	}

	/** Add handler function.
	 * @param method HTTP method to react to
	 * @param path path (after component base path) to react to
	 * @param handler handler function
	 */
	template <class O>
	void add_handler(WebRequest::Method method, std::string path,
	                 std::function<O (WebviewRestParams &)> handler)
	{
		routes_.push_back(std::make_tuple(method, path,
		                                  [this, handler](const std::string &body, WebviewRestParams& m)
		                                  -> std::unique_ptr<WebviewRestReply>
		                                  {
			                                  try {
				                                  O output{handler(m)};
				                                  try {
					                                  output.validate();
				                                  } catch (std::runtime_error &e) {
					                                  logger_->log_warn(("RestAPI|" + name_).c_str(), "%s", e.what());
				                                  }
				                                  return std::make_unique<WebviewRestReply>
					                                  (WebReply::HTTP_OK, output.to_json(pretty_json_ || m.pretty_json()));
			                                  } catch (WebviewRestException &e) {
				                                  return std::make_unique<WebviewRestReply>
					                                  (e.code(), e.what_no_backtrace());
			                                  } catch (Exception &e) {
				                                  auto r = std::make_unique<WebviewRestReply>
					                                  (WebReply::HTTP_INTERNAL_SERVER_ERROR);
				                                  r->append_body("Execution failed: %s", e.what_no_backtrace());
				                                  return r;
			                                  }
		                                  }));
	}

	/** Check if an actual path matches an API path pattern.
	 * @param url requested
	 * @param api_path configured API path to check
	 * @param params object to set argument mappings
	 * @return true if the path cold be matched, false otherwise.
	 */
	bool
	path_match(const std::string & url, const std::string & api_path, WebviewRestParams& params)
	{
		std::map<std::string, std::string> m;
		std::vector<std::string> url_s  = str_split(url, '/');
		std::vector<std::string> path_s = str_split(api_path, '/');
		if (url_s.size() != path_s.size())  return false;
		for (size_t i = 0; i < url_s.size(); ++i) {
			const std::string &p = path_s[i];
			const std::string &u = url_s[i];
			if (p.front() == '{' && p.back() == '}') {
				m[p.substr(1, p.length() - 2)] = u;
			} else if (p != u) {
				return false;
			}
		}
		params.set_path_args(std::move(m));
		return true;
	}

	/** Process REST API request.
	 * @param request incoming request
	 * @param rest_url the URL stripped of the base URL prefix
	 * @return reply
	 */
	WebReply *
	process_request(const WebRequest *request, const std::string & rest_url)
	{
		WebviewRestParams params;
		auto ri = std::find_if(routes_.begin(), routes_.end(),
			                       [this, rest_url, &params, request](auto r) {
			                       if (std::get<0>(r) != request->method()) {
				                       return false;
			                       }
			                       if (this->path_match(rest_url, std::get<1>(r), params)) {
				                       return true;
			                       } else {
				                       return false;
			                       }
		                       });
		if (ri == routes_.end()) {
			return new StaticWebReply(WebReply::HTTP_INTERNAL_SERVER_ERROR, "API " + rest_url + " unknown");
		}
		params.set_query_args(request->get_values());
		std::unique_ptr<WebviewRestReply> reply = std::get<2>(*ri)(request->body(), params);
		return reply.release();
	}

	/** Enable or disable pretty JSON printing globally.
	 * @param pretty true to enable
	 */
	void set_pretty_json(bool pretty)
	{
		pretty_json_ = pretty;
	}
	
 private:
	std::string     name_;
	fawkes::Logger *logger_;
	std::vector<std::tuple<WebRequest::Method, std::string, handler_func>> routes_;
	bool            pretty_json_;
};

}

#endif
