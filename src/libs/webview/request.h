
/***************************************************************************
 *  request.h - Web request
 *
 *  Created: Mon Jun 17 17:58:51 2013
 *  Copyright  2006-2014  Tim Niemueller [www.niemueller.de]
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

#include <map>
#include <string>

extern "C" {
  struct MHD_Connection;
  struct MHD_PostProcessor;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebRequest {
  friend class WebRequestDispatcher;

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
    METHOD_TRACE	///< TRACE
  } Method;

  WebRequest(const char *url, const char *method, MHD_Connection *connection);
  ~WebRequest();

  /** Get URL.
   * @return URL */
  const std::string &  url() const { return url_; } 
  /** Get HTTP transfer method.
   * @return request's HTTP transfer method */
  Method               method() const { return method_; } 

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

  /** Get raw post data.
   * @return raw port data or empty string if none. Note that this is not necesarily
   * a printable string (or zero-terminated) */
  const std::string &  raw_post_data() const { return post_raw_data_; }

 protected:
  /** Set cookie map.
   * @param cookies cookies map
   */
  void set_cookies(const std::map<std::string, std::string> &cookies) { cookies_ = cookies; }
  void set_raw_post_data(const char *data, size_t data_size);

 private:
  MHD_PostProcessor *pp_;

  std::string url_;
  Method      method_;
  std::map<std::string, std::string> cookies_;
  std::map<std::string, std::string> post_values_;
  std::string                        post_raw_data_;
  std::map<std::string, std::string> get_values_;
  std::map<std::string, std::string> headers_;
};


} // end namespace fawkes



#endif
