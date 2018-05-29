
/***************************************************************************
 *  reply.h - Web request reply
 *
 *  Created: Wed Oct 22 18:49:35 2008
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_WEBVIEW_REPLY_H_
#define __LIBS_WEBVIEW_REPLY_H_

#include <map>
#include <string>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class WebRequest;

class WebReply
{
 public:
  /** HTTP response code. */
  typedef enum {
    HTTP_CONTINUE                         = 100, /**< CONTINUE */
    HTTP_SWITCHING_PROTOCOLS              = 101, /**< SWITCHING_PROTOCOLS */
    HTTP_PROCESSING                       = 102, /**< PROCESSING */

    HTTP_OK                               = 200, /**< OK */
    HTTP_CREATED                          = 201, /**< CREATED */
    HTTP_ACCEPTED                         = 202, /**< ACCEPTED */
    HTTP_NON_AUTHORITATIVE_INFORMATION    = 203, /**< NON_AUTHORITATIVE_INFORMATION */
    HTTP_NO_CONTENT                       = 204, /**< NO_CONTENT */
    HTTP_RESET_CONTENT                    = 205, /**< RESET_CONTENT */
    HTTP_PARTIAL_CONTENT                  = 206, /**< PARTIAL_CONTENT */
    HTTP_MULTI_STATUS                     = 207, /**< MULTI_STATUS */

    HTTP_MULTIPLE_CHOICES                 = 300, /**< MULTIPLE_CHOICES */
    HTTP_MOVED_PERMANENTLY                = 301, /**< MOVED_PERMANENTLY */
    HTTP_FOUND                            = 302, /**< FOUND */
    HTTP_SEE_OTHER                        = 303, /**< SEE_OTHER */
    HTTP_NOT_MODIFIED                     = 304, /**< NOT_MODIFIED */
    HTTP_USE_PROXY                        = 305, /**< USE_PROXY */
    HTTP_SWITCH_PROXY                     = 306, /**< SWITCH_PROXY */
    HTTP_TEMPORARY_REDIRECT               = 307, /**< TEMPORARY_REDIRECT */

    HTTP_BAD_REQUEST                      = 400, /**< BAD_REQUEST */
    HTTP_UNAUTHORIZED                     = 401, /**< UNAUTHORIZED */
    HTTP_PAYMENT_REQUIRED                 = 402, /**< PAYMENT_REQUIRED */
    HTTP_FORBIDDEN                        = 403, /**< FORBIDDEN */
    HTTP_NOT_FOUND                        = 404, /**< NOT_FOUND */
    HTTP_METHOD_NOT_ALLOWED               = 405, /**< METHOD_NOT_ALLOWED */
    HTTP_METHOD_NOT_ACCEPTABLE            = 406, /**< METHOD_NOT_ACCEPTABLE */
    HTTP_PROXY_AUTHENTICATION_REQUIRED    = 407, /**< PROXY_AUTHENTICATION_REQUIRED */
    HTTP_REQUEST_TIMEOUT                  = 408, /**< REQUEST_TIMEOUT */
    HTTP_CONFLICT                         = 409, /**< CONFLICT */
    HTTP_GONE                             = 410, /**< GONE */
    HTTP_LENGTH_REQUIRED                  = 411, /**< LENGTH_REQUIRED */
    HTTP_PRECONDITION_FAILED              = 412, /**< PRECONDITION_FAILED */
    HTTP_REQUEST_ENTITY_TOO_LARGE         = 413, /**< REQUEST_ENTITY_TOO_LARGE */
    HTTP_REQUEST_URI_TOO_LONG             = 414, /**< REQUEST_URI_TOO_LONG */
    HTTP_UNSUPPORTED_MEDIA_TYPE           = 415, /**< UNSUPPORTED_MEDIA_TYPE */
    HTTP_REQUESTED_RANGE_NOT_SATISFIABLE  = 416, /**< REQUESTED_RANGE_NOT_SATISFIABLE */
    HTTP_EXPECTATION_FAILED               = 417, /**< EXPECTATION_FAILED */
    HTTP_UNPROCESSABLE_ENTITY             = 422, /**< UNPROCESSABLE_ENTITY */
    HTTP_LOCKED                           = 423, /**< LOCKED */
    HTTP_FAILED_DEPENDENCY                = 424, /**< FAILED_DEPENDENCY */
    HTTP_UNORDERED_COLLECTION             = 425, /**< UNORDERED_COLLECTION */
    HTTP_UPGRADE_REQUIRED                 = 426, /**< UPGRADE_REQUIRED */
    HTTP_RETRY_WITH                       = 449, /**< RETRY_WITH */

    HTTP_INTERNAL_SERVER_ERROR            = 500, /**< INTERNAL_SERVER_ERROR */
    HTTP_NOT_IMPLEMENTED                  = 501, /**< NOT_IMPLEMENTED */
    HTTP_BAD_GATEWAY                      = 502, /**< BAD_GATEWAY */
    HTTP_SERVICE_UNAVAILABLE              = 503, /**< SERVICE_UNAVAILABLE */
    HTTP_GATEWAY_TIMEOUT                  = 504, /**< GATEWAY_TIMEOUT */
    HTTP_HTTP_VERSION_NOT_SUPPORTED       = 505, /**< HTTP_VERSION_NOT_SUPPORTED */
    HTTP_VARIANT_ALSO_NEGOTIATES          = 506, /**< VARIANT_ALSO_NEGOTIATES */
    HTTP_INSUFFICIENT_STORAGE             = 507, /**< INSUFFICIENT_STORAGE */
    HTTP_BANDWIDTH_LIMIT_EXCEEDED         = 509, /**< BANDWIDTH_LIMIT_EXCEEDED */
    HTTP_NOT_EXTENDED                     = 510 /**< NOT_EXTENDED */
  } Code;

  /** Map of headers. */
  typedef std::map<std::string, std::string> HeaderMap;

  WebReply(Code code);
  virtual ~WebReply();

  Code              code() const;
  void              set_code(Code code);
  void              add_header(const std::string& header, const std::string& content);
  void              add_header(const std::string& header_string);
  const HeaderMap & headers() const;
  
  void              set_caching(bool caching);
  static void       set_caching_default(bool caching);

  void              set_request(WebRequest *request);
  WebRequest *      get_request() const;

  void              pack_caching();

 private:
  Code              code_;
  HeaderMap         headers_;
  bool              caching_;
  static bool       caching_default_;
  WebRequest       *request_;
};

class DynamicWebReply : public WebReply
{
 public:
  DynamicWebReply(Code code);

  virtual size_t chunk_size();
  virtual size_t size() = 0;
  virtual size_t next_chunk(size_t pos, char *buffer, size_t buf_max_size) = 0;
};

class StaticWebReply : public WebReply
{
 public:
  StaticWebReply(Code code, std::string body = "");

  void append_body(const char *format, ...);
  void append_body(const std::string &s);
  StaticWebReply & operator+=(std::string text);

  virtual const std::string & body();
  virtual std::string::size_type body_length();

  virtual void pack();
 protected:
  /** Body of the reply. */
  std::string _body;
};

extern WebReply * no_caching(WebReply *reply);

} // end namespace fawkes

#endif
