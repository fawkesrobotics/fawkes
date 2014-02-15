
/***************************************************************************
 *  request.cpp - Web request
 *
 *  Created: Mon Jun 17 18:04:04 2013
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

#include <core/exception.h>
#include <webview/request.h>
#include <sys/types.h>
#include <sys/select.h>
#include <unistd.h>
#include <stdint.h>
#include <microhttpd.h>
#include <cstring>

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
 * @param url requested URL
 * @param method HTTP transfer method
 * @param connection MicroHTTPd connection
 */
WebRequest::WebRequest(const char *url, const char *method, MHD_Connection *connection)
  : pp_(NULL), url_(url)
{
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
  }

  MHD_get_connection_values(connection, MHD_HEADER_KIND, &header_iterator, this);
  MHD_get_connection_values(connection, MHD_COOKIE_KIND, &cookie_iterator, this);
  MHD_get_connection_values(connection,
			    MHD_GET_ARGUMENT_KIND, &get_argument_iterator, this);
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


/** Set raw post data.
 * The data is copied as is without assuming a human-readable string
 * or even just zero-termination.
 * @param data data to copy
 * @param data_size size in bytes of \@p data
 */
void
WebRequest::set_raw_post_data(const char *data, size_t data_size)
{
  post_raw_data_ = std::string(data, data_size);
}


} // end namespace fawkes
