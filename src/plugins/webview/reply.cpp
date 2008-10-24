
/***************************************************************************
 *  reply.cpp - Web request reply
 *
 *  Created: Thu Oct 23 12:01:05 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include "reply.h"

#include <cstdlib>
#include <cstdarg>

/** @class WebReply "reply.h"
 * Basic web reply.
 * The base class for all web replies. Though the WebRequestDispatcher expects
 * sub-classes of StaticWebReply or DynamicWebReply.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param code HTTP response code
 */
WebReply::WebReply(response_code_t code)
{
  __code = code;
}


/** Destructor. */
WebReply::~WebReply()
{
}


/** Get response code.
 * @return HTTP response code
 */
WebReply::response_code_t
WebReply::code() const
{
  return __code;
}


/** Add a HTTP header.
 * @param header header entry name
 * @param content content of the header field
 */
void
WebReply::add_header(std::string header, std::string content)
{
  __headers[header] = content;
}


/** get headers.
 * @return map of header name/content pairs.
 */
const WebReply::HeaderMap &
WebReply::headers() const
{
  return __headers;
}


/** @class DynamicWebReply "reply.h"
 * Dynamic web reply.
 * A reply of this type is send out in chunks, not all as a whole. It should be
 * used for payloads that can get very large, like file transfers.
 * @author Tim Niemueller
 *
 * @fn size_t DynamicWebReply::size() = 0
 * Total size of the web reply.
 * Return the total size of the reply if known, or 0 if it is not known. In the
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
DynamicWebReply::DynamicWebReply(response_code_t code)
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


/** @class StaticWebReply "reply.h"
 * Static web reply.
 * The static web reply is send out as a whole at once and is immediately
 * deleted after sending. Use it for regular-sized pages and content.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param code HTTP response code
 * @param body optional initial body
 */
StaticWebReply::StaticWebReply(response_code_t code, std::string body)
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
