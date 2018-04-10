
/***************************************************************************
 *  rest_api.h - Webview REST API
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

#ifndef __LIBS_WEBVIEW_REST_API_H_
#define __LIBS_WEBVIEW_REST_API_H_

#include <webview/request.h>
#include <webview/reply.h>

#include <core/exception.h>
#include <logging/logger.h>
#include <utils/misc/string_split.h>

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <functional>
#include <algorithm>
#include <regex>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

template <typename T> class WebviewRouter;

/** REST reply via Webview.
 * @author Tim Niemueller
 */
class WebviewRestReply : public StaticWebReply
{
 public:
	/** Constructor.
	 * @param code HTTP response code
	 * @param body body of reply, usually a JSON document.
	 * @param content_type content type of reply, defaults to application/json.
	 * When sending text error messages, set to text/plain.
	 */
 WebviewRestReply(WebReply::Code code, const std::string &body = "",
                  const std::string &content_type = "application/json")
		: StaticWebReply(code, body)
	{
		add_header("Content-type", content_type);
	}
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
	explicit
	WebviewRestException(WebReply::Code code, const char *format, ...)
		: Exception(), code_(code), content_type_("text/plain")
	{
		va_list va;
		va_start(va, format);
		append_va(format, va);
		va_end(va);
	}

	/** Constructor.
	 * @param code HTTP response code
	 * @param o Object to convert to JSON
	 * @param pretty true to enable pretty printing of the JSON input
	 */
	template<typename T,
		typename = std::enable_if_t<std::is_class<T>::value>>
	WebviewRestException(WebReply::Code code, const T& o,
		                     bool pretty = false)
	 : Exception(), code_(code), content_type_("application/json")
	{
		append("%s", o.to_json(pretty).c_str());	
	}

	/** Get HTTP response code.
	 * @return HTTP response code
	 */
	WebReply::Code code()
	{
		return code_;
	}

	/** Get content type of response.
	 * @return HTTP content type
	 */
	const std::string &
	content_type() const {
		return content_type_;
	}

 private:
	WebReply::Code code_;
	std::string content_type_;
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

	/** Check if query argument is set.
	 * Retrieves a named query argument that was passed in the
	 * URL, e.g., retrieve "pretty" for "?pretty".
	 * @param what what to check
	 * @return true if the argument exists (with any value), false otherwise
	 */
	bool has_query_arg(const std::string& what)
	{
		return (query_args_.find(what) != query_args_.end());
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

class Logger;

class WebviewRestApi
{
 public:
	WebviewRestApi(const std::string &name, fawkes::Logger *logger);

	/** REST API call handler function type. */
	typedef std::function<std::unique_ptr<WebReply> (std::string, WebviewRestParams&)> Handler;

	const std::string & 	name() const;
	void add_handler(WebRequest::Method method, std::string path, Handler handler);
	void set_pretty_json(bool pretty);

	/** Add simple handler.
	 * For a handler that does not require input parameters and that outputs
	 * a WebviewRestReply instance, for example, only to indicate success.
	 * @param method HTTP method to react to
	 * @param path path (after component base path) to react to
	 * @param handler handler function
	 */
	void add_handler(WebRequest::Method method, std::string path,
	                 std::function<std::unique_ptr<WebReply> (WebviewRestParams &)> handler)
	{
		add_handler(method, path,
		            [this, handler](const std::string &body, WebviewRestParams& m)
		            -> std::unique_ptr<WebReply>
		            {
			            try {
				            return handler(m);
			            } catch (WebviewRestException &e) {
				            return std::make_unique<WebviewRestReply>
					            (e.code(), e.what_no_backtrace(), e.content_type());
			            } catch (Exception &e) {
				            auto r = std::make_unique<WebviewRestReply>
					            (WebReply::HTTP_INTERNAL_SERVER_ERROR);
				            r->append_body("Execution failed: %s", e.what_no_backtrace());
				            r->add_header("Content-type", "text/plain");
				            return r;
			            }
		            });
	}

	/** Add handler function.
	 * @param method HTTP method to react to
	 * @param path path (after component base path) to react to
	 * @param handler handler function
	 */
	template <class O, class I>
	void
	add_handler(WebRequest::Method method, std::string path,
		            std::function<O (I, WebviewRestParams &)> handler)
	{
		add_handler(method, path,
		            [this, handler](const std::string &body, WebviewRestParams& m)
		            -> std::unique_ptr<WebReply>
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
				            if (m.has_query_arg("pretty")) {
					            m.set_pretty_json(true);
				            }
				            return std::make_unique<WebviewRestReply>
					            (WebReply::HTTP_OK, output.to_json(pretty_json_ || m.pretty_json()));
			            } catch (WebviewRestException &e) {
				            return std::make_unique<WebviewRestReply>
					            (e.code(), e.what_no_backtrace(), e.content_type());
			            } catch (Exception &e) {
				            auto r = std::make_unique<WebviewRestReply>
					            (WebReply::HTTP_INTERNAL_SERVER_ERROR);
				            r->append_body("Execution failed: %s", e.what_no_backtrace());
				            r->add_header("Content-type", "text/plain");
				            return r;
			            }
		            });
	}

	/** Add handler function.
	 * @param method HTTP method to react to
	 * @param path path (after component base path) to react to
	 * @param handler handler function
	 */
	template <class I>
	void add_handler(WebRequest::Method method, std::string path,
	                 std::function<std::unique_ptr<WebReply> (I, WebviewRestParams &)> handler)
	{
		add_handler(method, path,
		            [this, handler](const std::string &body, WebviewRestParams& m)
		            -> std::unique_ptr<WebReply>
		            {
			            I input;
			            input.from_json(body);
			            try {
				            return handler(std::forward<I>(input), m);
			            } catch (WebviewRestException &e) {
				            return std::make_unique<WebviewRestReply>
					            (e.code(), e.what_no_backtrace(), e.content_type());
			            } catch (Exception &e) {
				            auto r = std::make_unique<WebviewRestReply>
					            (WebReply::HTTP_INTERNAL_SERVER_ERROR);
				            r->append_body("Execution failed: %s", e.what_no_backtrace());
				            r->add_header("Content-type", "text/plain");
				            return r;
			            }
		            });
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
		add_handler(method, path,
		            [this, handler](const std::string &body, WebviewRestParams& m)
		            -> std::unique_ptr<WebReply>
		            {
			            try {
				            O output{handler(m)};
				            try {
					            output.validate();
				            } catch (std::runtime_error &e) {
					            logger_->log_warn(("RestAPI|" + name_).c_str(), "%s", e.what());
				            }
				            if (m.has_query_arg("pretty")) {
					            m.set_pretty_json(true);
				            }
				            return std::make_unique<WebviewRestReply>
					            (WebReply::HTTP_OK, output.to_json(pretty_json_ || m.pretty_json()));
			            } catch (WebviewRestException &e) {
				            return std::make_unique<WebviewRestReply>
					            (e.code(), e.what_no_backtrace(), e.content_type());
			            } catch (Exception &e) {
				            auto r = std::make_unique<WebviewRestReply>
					            (WebReply::HTTP_INTERNAL_SERVER_ERROR);
				            r->append_body("Execution failed: %s", e.what_no_backtrace());
				            r->add_header("Content-type", "text/plain");
				            return r;
			            }
		            });
	}

	WebReply *
		process_request(const WebRequest *request, const std::string &rest_url);

 private:
	std::string     name_;
	fawkes::Logger *logger_;
	bool            pretty_json_;
	std::shared_ptr<WebviewRouter<Handler>> router_;
};

}

#endif
