
/***************************************************************************
 *  reply.cpp - Web request reply
 *
 *  Created: Thu Oct 23 12:01:05 2008
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

#include <webview/reply.h>

#include <core/exception.h>
#include <cstdlib>
#include <cstdarg>
#include <cstdio>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Disable caching on a reply.
 * This is a convenience wrapper to reply->set_caching(false). It enables
 * the following call styles:
 * @code
 * return no_caching(new StaticWebReply(Reply::HTTP_NOT_FOUND, "Not Found"));
 *
 * return no_caching(some_handler());
 * @endcode
 * This works on any reply without always patching a boolean flag into
 * the ctor and without first storing the pointer, calling the function,
 * and then returning.
 * @param reply reply to disable caching for
 * @return this
 */
WebReply *
no_caching(WebReply *reply)
{
	reply->set_caching(false);
	return reply;
}


/** @class WebReply <webview/reply.h>
 * Basic web reply.
 * The base class for all web replies. Though the WebRequestDispatcher expects
 * sub-classes of StaticWebReply or DynamicWebReply.
 * @author Tim Niemueller
 */

/// Enable caching for this reply?
bool WebReply::caching_default_ = true;

/** Constructor.
 * @param code HTTP response code
 */
WebReply::WebReply(Code code)
{
  code_ = code;
  request_ = NULL;

  caching_ = caching_default_;
}


/** Destructor. */
WebReply::~WebReply()
{
}


/** Enable or disable caching default for replies.
 * This static setting controls whether following replies will allow
 * for client-side of the web pages or not by default. Disabling this
 * allows to force clients to always reload the pages.
 * @param caching true to enable client-side caching, false to disable
 */
void
WebReply::set_caching_default(bool caching)
{
  caching_default_ = caching;
}

/** Enable or disable caching for this specific reply.
 * @param caching true to enable client-side caching, false to disable
 */
void
WebReply::set_caching(bool caching)
{
  caching_ = caching;
}


/** Get response code.
 * @return HTTP response code
 */
WebReply::Code
WebReply::code() const
{
  return code_;
}

/** Set response code.
 * @param code HTTP response code
 */
void
WebReply::set_code(WebReply::Code code)
{
	code_ = code;
}

/** Add a HTTP header.
 * @param header header entry name
 * @param content content of the header field
 */
void
WebReply::add_header(const std::string& header, const std::string& content)
{
  headers_[header] = content;
}


/** Add a HTTP header.
 * @param header_string header string of the format "Key: Value".
 */
void
WebReply::add_header(const std::string& header_string)
{
  std::string::size_type pos;
  if ((pos = header_string.find(":")) != std::string::npos) {
    std::string header = header_string.substr(0, pos);
    std::string content;
    if (header_string[pos+1] == ' ') {
      content = header_string.substr(pos+2);
    } else {
      content = header_string.substr(pos+1);
    }
    headers_[header] = content;
  } else {
    throw Exception("Invalid header '%s'", header_string.c_str());
  }
}


/** get headers.
 * @return map of header name/content pairs.
 */
const WebReply::HeaderMap &
WebReply::headers() const
{
  return headers_;
}


/** Get associated request.
 * This is only valid after set_request() has been called.
 * @return associated web request
 */
WebRequest *
WebReply::get_request() const
{
  return request_;
}


/** Set associated request.
 * @param request associated request
 */
void
WebReply::set_request(WebRequest *request)
{
  request_ = request;
}

/** Called just before the reply is sent.
 * Sets no-caching flags if caching has been disabled. 
 */
void
WebReply::pack_caching()
{
  if (! caching_) {
    // Headers to disable caching
    headers_["Cache-Control"] = "no-cache, no-store, must-revalidate, max-age=0";
  }
}

/** @class DynamicWebReply <webview/reply.h>
 * Dynamic web reply.
 * A reply of this type is send out in chunks, not all as a whole. It should be
 * used for payloads that can get very large, like file transfers.
 * @author Tim Niemueller
 *
 * @fn size_t DynamicWebReply::size() = 0
 * Total size of the web reply.
 * Return the total size of the reply if known, or -1 if it is not known. In the
 * latter case your next_chunk() method has to return -1 at some point to end
 * the transfer. If possible by any means return a meaningful value, as it will
 * improve the experience of users, especially for long transfers!
 * @return total size of reply in bytes
 *
 * @fn size_t DynamicWebReply::next_chunk(size_t pos, char *buffer, size_t buf_max_size) = 0
 * Get data of next chunk.
 * @param pos position in the stream. Note that a certain position may be called
 * several times.
 * @param buffer buffer to store data in
 * @param buf_max_size maximum size in bytes of data that can be put into buffer
 * @return number of bytes written to buffer, or -1 to immediately stop the
 * transfer.
 */

/** Constructor.
 * @param code HTTP response code
 */
DynamicWebReply::DynamicWebReply(Code code)
  : WebReply(code)
{
}


/** Chunksize.
 * The size that a single chunk should have. A sub-class may override this if a
 * specific chunk size is beneficial or even required. The default is 32kb.
 * @return chunk size in bytes
 */
size_t
DynamicWebReply::chunk_size()
{
  // use 32k chunks by default
  return 32 * 1024;
}


/** @class StaticWebReply <webview/reply.h>
 * Static web reply.
 * The static web reply is send out as a whole at once and is immediately
 * deleted after sending. Use it for regular-sized pages and content.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param code HTTP response code
 * @param body optional initial body
 */
StaticWebReply::StaticWebReply(Code code, std::string body)
  : WebReply(code)
{
  _body = body;
}


/** Append to body.
 * @param format format of the text to append. Supports the same format as
 * printf().
 */
void
StaticWebReply::append_body(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  char *s;
  if ( vasprintf(&s, format, args) != -1 ) {
    _body += s;
    free(s);
  }
  va_end(args);
}

/** Append string to body.
 * @param s string to add, this may contain null bytes
 */
void
StaticWebReply::append_body(const std::string &s)
{
	_body += s;
}


/** Append simple text line.
 * @param text text to append to body
 * @return reference to this instance
 */
StaticWebReply &
StaticWebReply::operator+=(std::string text)
{
  _body += text;
  return *this;
}


/** Get body.
 * @return reference to body.
 */
const std::string &
StaticWebReply::body()
{
  return _body;
}


/** Get length of body.
 * @return body length
 */
std::string::size_type
StaticWebReply::body_length()
{
  return _body.length();
}


/** Pack the data.
 * This method is called just before the reply is sent.
 * You can implement this method if you need to compose your reply before
 * body() and body_length() provide valid output.
 */
void
StaticWebReply::pack()
{
}

} // end namespace fawkes
