
/***************************************************************************
 *  request.cpp - Web request
 *
 *  Created: Mon Jun 17 18:04:04 2013
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

#include <core/exception.h>
#include <webview/request.h>
#include <sys/types.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdint.h>
#include <microhttpd.h>
#include <cstring>
#include <netinet/in.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond INTERNAL
static int
cookie_iterator(void *cls, enum MHD_ValueKind kind,
		const char *key, const char *value)
{
  WebRequest *request = static_cast<WebRequest *>(cls);
  request->set_cookie(key, value);
  return MHD_YES;
}

static int
get_argument_iterator(void *cls, enum MHD_ValueKind kind,
		      const char *key, const char *value)
{
  WebRequest *request = static_cast<WebRequest *>(cls);
  if (value == NULL)  request->set_get_value(key, "");
  else                request->set_get_value(key, value);
  return MHD_YES;
}

static int
header_iterator(void *cls, enum MHD_ValueKind kind,
		const char *key, const char *value)
{
  WebRequest *request = static_cast<WebRequest *>(cls);
  if (value == NULL)  request->set_header(key, "");
  else                request->set_header(key, value);
  return MHD_YES;
}
/// @endcond

/** @class WebRequest <webview/request.h>
 * Web request meta data carrier.
 * For incoming web requests this class is instantiate to carry the
 * necessary information for carriers like URL, request method,
 * or cookie and POST form values.
 * @author Tim Niemueller
 */


/** Constructor.
 * @param uri URI of the request
 */
WebRequest::WebRequest(const char *uri)
  : pp_(NULL), is_setup_(false), uri_(uri)
{
  reply_size_ = 0;
}

/** Complete setting up of request.
 * @param url requested URL
 * @param method HTTP transfer method
 * @param version HTTP version string
 * @param connection MicroHTTPd connection
 */
void
WebRequest::setup(const char *url, const char *method,
		  const char *version, MHD_Connection *connection)
{
  url_ = url;

  if (0 == strcmp(method, MHD_HTTP_METHOD_GET)) {
    method_ = METHOD_GET;
  } else if (0 == strcmp(method, MHD_HTTP_METHOD_POST)) {
    method_ = METHOD_POST;
  } else if (0 == strcmp(method, MHD_HTTP_METHOD_HEAD)) {
    method_ = METHOD_HEAD;
  } else if (0 == strcmp(method, MHD_HTTP_METHOD_DELETE)) {
    method_ = METHOD_DELETE;
  } else if (0 == strcmp(method, MHD_HTTP_METHOD_PUT)) {
    method_ = METHOD_PUT;
  } else if (0 == strcmp(method, MHD_HTTP_METHOD_CONNECT)) {
    method_ = METHOD_CONNECT;
  } else if (0 == strcmp(method, MHD_HTTP_METHOD_OPTIONS)) {
    method_ = METHOD_OPTIONS;
  } else if (0 == strcmp(method, MHD_HTTP_METHOD_TRACE)) {
    method_ = METHOD_TRACE;
  } else if (0 == strcmp(method, MHD_HTTP_METHOD_PATCH)) {
    method_ = METHOD_PATCH;
  }

  if (0 == strcmp(version, MHD_HTTP_VERSION_1_0)) {
    http_version_ = HTTP_VERSION_1_0;
  } else if (0 == strcmp(version, MHD_HTTP_VERSION_1_1)) {
    http_version_ = HTTP_VERSION_1_1;
  }

  MHD_get_connection_values(connection, MHD_HEADER_KIND, &header_iterator, this);
  MHD_get_connection_values(connection, MHD_COOKIE_KIND, &cookie_iterator, this);
  MHD_get_connection_values(connection,
			    MHD_GET_ARGUMENT_KIND, &get_argument_iterator, this);


  // check for reverse proxy header fields
  if (headers_.find("X-Forwarded-For") != headers_.end()) {
	  std::string forwarded_for{headers_["X-Forwarded-For"]};
	  std::string::size_type comma_pos = forwarded_for.find(",");
	  if (comma_pos != std::string::npos) {
		  forwarded_for = forwarded_for.substr(0, comma_pos);
	  }
	  client_addr_ = forwarded_for;

  } else {
	  struct sockaddr *client_addr =
		  MHD_get_connection_info(connection, MHD_CONNECTION_INFO_CLIENT_ADDRESS)
		  ->client_addr;

	  char addr_str[INET6_ADDRSTRLEN];
	  switch(client_addr->sa_family) {
	  case AF_INET:
		  inet_ntop(AF_INET, &(((struct sockaddr_in *)client_addr)->sin_addr),
		            addr_str, INET6_ADDRSTRLEN);
		  break;

	  case AF_INET6:
		  inet_ntop(AF_INET6, &(((struct sockaddr_in6 *)client_addr)->sin6_addr),
		            addr_str, INET6_ADDRSTRLEN);
		  break;

	  default:
		  strncpy(addr_str, "Unknown AF", INET6_ADDRSTRLEN);
	  }

	  client_addr_ = addr_str;
  }

  is_setup_ = true;
}


/** Destructor. */
WebRequest::~WebRequest()
{
  if (pp_) {
    MHD_destroy_post_processor(pp_);
    pp_ = NULL;
  }
}


/** Set a POST value.
 * @param key key of the value
 * @param data data of the value
 * @param size size in bytes of @p data
 */
void
WebRequest::set_post_value(const char *key, const char *data, size_t size)
{
  std::string val_add(data, size);
  if (post_values_.find(key) != post_values_.end()) {
    post_values_[key] += val_add;
  } else {
    post_values_[key]  = val_add;
  }
}


/** Set request body.
 * The data is copied as is without assuming a human-readable string
 * or even just zero-termination.
 * @param data data to copy
 * @param data_size size in bytes of \@p data
 */
void
WebRequest::set_body(const char *data, size_t data_size)
{
  body_ = std::string(data, data_size);
}

/** Add to request body.
 * The data is copied as is without assuming a human-readable string
 * or even just zero-termination.
 * @param data data to copy
 * @param data_size size in bytes of \@p data
 */
void
WebRequest::addto_body(const char *data, size_t data_size)
{
  body_ += std::string(data, data_size);
}

/** Finalize body handling.
 * Check for zero termination of body, and if it does not exist, add it.
 */
void
WebRequest::finish_body()
{
	if (body_.length() == 0)  return;
	if (body_[body_.length()-1] != 0) {
		body_ += '\0';
	}
}

/** Increment reply bytes counter.
 * @param increment_by number of bytes sent
 */
void
WebRequest::increment_reply_size(size_t increment_by)
{
  reply_size_ += increment_by;
}

/** Get number of bytes actually sent out so far.
 * @return number of bytes sent
 */
size_t
WebRequest::reply_size() const
{
  return reply_size_;
}


/** Get method as string.
 * @return HTTP method as string
 */
const char *
WebRequest::method_str() const
{
  switch (method_) {
  case METHOD_CONNECT: return MHD_HTTP_METHOD_CONNECT;
  case METHOD_DELETE:  return MHD_HTTP_METHOD_DELETE;
  case METHOD_GET:     return MHD_HTTP_METHOD_GET;
  case METHOD_HEAD:    return MHD_HTTP_METHOD_HEAD;
  case METHOD_OPTIONS: return MHD_HTTP_METHOD_OPTIONS;
  case METHOD_POST:    return MHD_HTTP_METHOD_POST;
  case METHOD_PUT:     return MHD_HTTP_METHOD_PUT;
  case METHOD_TRACE:   return MHD_HTTP_METHOD_TRACE;
  default: return "UNKNOWN_METHOD";
  }
}


/** Get HTTP version as string.
 * @return HTTP version as string.
 */
const char *
WebRequest::http_version_str() const
{
  switch (http_version_) {
  case HTTP_VERSION_1_0: return MHD_HTTP_VERSION_1_0;
  case HTTP_VERSION_1_1: return MHD_HTTP_VERSION_1_1;
  default: return "UNKNOWN_VERSION";
  }
}


/** Set HTTP code of the final reply.
 * @param code reply code
 */
void
WebRequest::set_reply_code(WebReply::Code code)
{
  reply_code_ = code;
}


/** Get HTTP code of reply.
 * @return HTTP code of reply
 */
WebReply::Code
WebRequest::reply_code() const
{
  return reply_code_;
}

} // end namespace fawkes
