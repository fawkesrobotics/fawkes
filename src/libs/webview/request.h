
/***************************************************************************
 *  request.h - Web request
 *
 *  Created: Mon Jun 17 17:58:51 2013
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

#ifndef __LIBS_WEBVIEW_REQUEST_H_
#define __LIBS_WEBVIEW_REQUEST_H_

#include <webview/reply.h>
#include <utils/time/time.h>

#include <map>
#include <string>
#include <arpa/inet.h>

extern "C" {
  struct MHD_Connection;
  struct MHD_PostProcessor;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebRequestDispatcher;

class WebRequest {
  friend WebRequestDispatcher;

 public:
  /** HTTP transfer methods. */
  typedef enum {
    METHOD_CONNECT,	///< CONNECT
    METHOD_DELETE,	///< DELETE
    METHOD_GET,		///< GET
    METHOD_HEAD,	///< HEAD
    METHOD_OPTIONS,	///< OPTIONS
    METHOD_POST,	///< POST
    METHOD_PUT,		///< PUT
    METHOD_TRACE,	///< TRACE
    METHOD_PATCH	///< PATCH
  } Method;

  /** HTTP version. */
  typedef enum {
    HTTP_VERSION_1_0,
    HTTP_VERSION_1_1
  } HttpVersion;

  WebRequest(const char *uri);
  ~WebRequest();

  /** Get URL.
   * @return URL */
  const std::string &  url() const { return url_; } 

  /** Get URI.
   * @return URI */
  const std::string &  uri() const { return uri_; } 

  /** Get HTTP transfer method.
   * @return request's HTTP transfer method */
  Method               method() const { return method_; } 
  const char *         method_str() const;

  /** Get HTTP version.
   * @return HTTP protocol version */
  HttpVersion          http_version() const { return http_version_; } 
  const char *         http_version_str() const;

  /** Get request time.
   * @return request time */
  const Time &         time() const { return time_; }

  /** Get name of authenticated user (basic auth).
   * @return name of authenticated user or empty if non-protected URL */
  const std::string &  user() const { return user_; } 

  /** Get client address as string.
   * @return client address as string */
  const std::string &  client_addr() const { return client_addr_; } 

  /** Get map of cookies.
   * @return map of cookies. */
  const std::map<std::string, std::string> &  cookies() const { return cookies_; }
  /** Get specific cookie.
   * @param key key of the cookie
   * @return value of cookie or empty string if not set
   */
  std::string  cookie(std::string &key) const
  {
    std::map<std::string, std::string>::const_iterator c = cookies_.find(key);
    return (c != cookies_.end()) ? c->second : "";
  }
  /** Check if the named cookie has been received.
   * @param key key of the requested cookie
   * @return true if the cookie was set, false otherwise
   */
  bool has_cookie(std::string key) const
  { return (cookies_.find(key) != cookies_.end()); }

  /** Get map of POST values.
   * @return map of POST values. */
  const std::map<std::string, std::string> &  post_values() const { return post_values_; }
  /** Get specific POST value.
   * @param key key of the post value
   * @return value of post value or empty string if not set
   */
  std::string  post_value(std::string &key) const
  {
    std::map<std::string, std::string>::const_iterator p = post_values_.find(key);
    return (p != post_values_.end()) ? p->second : "";
  }
  /** Get specific POST value.
   * @param key key of the post value
   * @return value of post value or empty string if not set
   */
  std::string  post_value(const char *key) const
  {
    std::map<std::string, std::string>::const_iterator p = post_values_.find(key);
    return (p != post_values_.end()) ? p->second : "";
  }
  /** Check if the named post value has been received.
   * @param key key of the post value
   * @return true if the post value was received, false otherwise
   */
  bool has_post_value(std::string key) const
  { return (post_values_.find(key) != post_values_.end()); }

  /** Get map of GET values.
   * @return map of GET values. */
  const std::map<std::string, std::string> &  get_values() const { return get_values_; }
  /** Get specific GET value.
   * @param key key of the get value
   * @return value of get value or empty string if not set
   */
  std::string  get_value(std::string &key) const
  {
    std::map<std::string, std::string>::const_iterator p = get_values_.find(key);
    return (p != get_values_.end()) ? p->second : "";
  }
  /** Get specific GET value.
   * @param key key of the get value
   * @return value of get value or empty string if not set
   */
  std::string  get_value(const char *key) const
  {
    std::map<std::string, std::string>::const_iterator p = get_values_.find(key);
    return (p != get_values_.end()) ? p->second : "";
  }
  /** Check if the named get value has been received.
   * @param key key of the requested get value
   * @return true if the get value was received, false otherwise
   */
  bool has_get_value(std::string key) const
  { return (get_values_.find(key) != get_values_.end()); }


  /** Get map of header values.
   * @return map of header values. */
  const std::map<std::string, std::string> &  headers() const { return headers_; }
  /** Header specific header value.
   * @param key key of the header value
   * @return value of header value or empty string if not set
   */
  std::string  header(std::string &key) const
  {
    std::map<std::string, std::string>::const_iterator p = headers_.find(key);
    return (p != headers_.end()) ? p->second : "";
  }
  /** Get specific header value.
   * @param key key of the header value
   * @return value of header value or empty string if not set
   */
  std::string  header(const char *key) const
  {
    std::map<std::string, std::string>::const_iterator p = headers_.find(key);
    return (p != headers_.end()) ? p->second : "";
  }
  /** Check if the named header value has been received.
   * @param key key of the requested header
   * @return true if the header value was received, false otherwise
   */
  bool has_header(std::string key) const
  { return (headers_.find(key) != headers_.end()); }

  /** Set a cookie.
   * @param key key of the cookie
   * @param value value of the cookie
   */
  void set_cookie(const std::string &key, const std::string &value) { cookies_[key] = value; }

  /** Set a POST value.
   * @param key key of the cookie
   * @param data incoming data
   * @param size size in bytes of @p data
   */
  void set_post_value(const char *key, const char *data, size_t size);

  /** Set a GET value.
   * @param key key of the cookie
   * @param value value of the GET argument
   */
  void set_get_value(const std::string &key, const std::string &value) { get_values_[key] = value; }

  /** Set a header value.
   * @param key key of the cookie
   * @param value value of the header argument
   */
  void set_header(const std::string &key, const std::string &value)
  { headers_[key] = value; }

  /** Get a path argument.
	 * Retrieves a named argument that was a token in the
	 * registration URL, e.g., retrieve "id" for "/item/{id}".
	 * @param what what to retrieve
	 * @return item passed in URL or empty string
	 */
	std::string path_arg(const std::string& what) const
	{
		const auto p = path_args_.find(what);
		if (p != path_args_.end()) {
			return p->second;
		} else {
			return "";
		}
	}

	/** Set path arguments.
	 * @param args path arguments
	 */
	void set_path_args(std::map<std::string, std::string>&& args)
	{
		path_args_ = std::move(args);
	}

  /** Get body of request.
   * @return The data that was received with the request. This is not
   * set if we receive a form submission. The values will be available
   * as POST values then. Note that this is not necesarily a printable
   * string (or zero-terminated) */
  const std::string &  body() const { return body_; }

  void   increment_reply_size(size_t increment_by);
  size_t reply_size() const;
  WebReply::Code reply_code() const;
  void           set_reply_code(WebReply::Code code);

 protected:
  /** Set cookie map.
   * @param cookies cookies map
   */
  void set_cookies(const std::map<std::string, std::string> &cookies) { cookies_ = cookies; }
  void set_body(const char *data, size_t data_size);
  void addto_body(const char *data, size_t data_size);
  void finish_body();

 private:
  bool is_setup() { return is_setup_; }
  void setup(const char *url, const char *method,
	     const char *version, MHD_Connection *connection);

 private:
  MHD_PostProcessor *pp_;
  bool is_setup_;

  std::string uri_;
  std::string url_;
  std::string user_;
  std::string client_addr_;
  Method      method_;
  HttpVersion http_version_;
  Time        time_;
  size_t      reply_size_;
  WebReply::Code reply_code_;
  std::map<std::string, std::string> cookies_;
  std::map<std::string, std::string> post_values_;
  std::string                        body_;
  std::map<std::string, std::string> get_values_;
  std::map<std::string, std::string> headers_;
	std::map<std::string, std::string> path_args_;
};


} // end namespace fawkes



#endif
