
/***************************************************************************
 *  error_reply.h - Web request reply for an error page
 *
 *  Created: Fri Oct 24 19:57:19 2008
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

#include <webview/error_reply.h>

#include <core/exceptions/software.h>
#include <cstdio>
#include <cstdlib>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebErrorPageReply <webview/error_reply.h>
 * Static error page reply.
 * Shows a simple error page based on the given code.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param code error code, must be a 4xx or 5xx HTTP code
 * @param format format for additional error message, use format as
 * known from sprintf.
 */
WebErrorPageReply::WebErrorPageReply(Code code,
				     const char *format, ...)
  : WebPageReply(code)
{
  if ( ((int)code < 400) || ((int)code > 599) ) {
    throw fawkes::OutOfBoundsException("Error code invalid", code, 400, 599);
  }

  switch (code) {
  case HTTP_BAD_REQUEST:
    _title = "400 BAD_REQUEST";
    _body  = "<h1>400 BAD_REQUEST</h1>";
    break;

  case HTTP_UNAUTHORIZED:
    _title = "401 UNAUTHORIZED";
    _body  = "<h1>401 UNAUTHORIZED</h1>";
    break;

  case HTTP_PAYMENT_REQUIRED:
    _title = "402 PAYMENT_REQUIRED";
    _body  = "<h1>402 PAYMENT_REQUIRED</h1>";
    break;

  case HTTP_FORBIDDEN:
    _title = "403 FORBIDDEN";
    _body  = "<h1>403 FORBIDDEN</h1>";
    break;

  case HTTP_NOT_FOUND:
    _title = "404 NOT_FOUND";
    _body  = "<h1>404 NOT_FOUND</h1>";    break;

  case HTTP_METHOD_NOT_ALLOWED:
    _title = "405 METHOD_NOT_ALLOWED";
    _body  = "<h1>405 METHOD_NOT_ALLOWED</h1>";
    break;

  case HTTP_METHOD_NOT_ACCEPTABLE:
    _title = "406 METHOD_NOT_ACCEPTABLE";
    _body  = "<h1>406 METHOD_NOT_ACCEPTABLE</h1>";
    break;

  case HTTP_PROXY_AUTHENTICATION_REQUIRED:
    _title = "407 PROXY_AUTHENTICATION_REQUIRED";
    _body  = "<h1>407 PROXY_AUTHENTICATION_REQUIRED</h1>";
    break;

  case HTTP_REQUEST_TIMEOUT:
    _title = "408 REQUEST_TIMEOUT";
    _body  = "<h1>408 REQUEST_TIMEOUT</h1>";
    break;

  case HTTP_CONFLICT:
    _title = "409 CONFLICT";
    _body  = "<h1>409 CONFLICT</h1>";
    break;

  case HTTP_GONE:
    _title = "410 GONE";
    _body  = "<h1>410 GONE</h1>";
    break;

  case HTTP_LENGTH_REQUIRED:
    _title = "411 LENGTH_REQUIRED";
    _body  = "<h1>411 LENGTH_REQUIRED</h1>";
    break;

  case HTTP_PRECONDITION_FAILED:
    _title = "412 PRECONDITION_FAILED";
    _body  = "<h1>412 PRECONDITION_FAILED</h1>";
    break;

  case HTTP_REQUEST_ENTITY_TOO_LARGE:
    _title = "413 REQUEST_ENTITY_TOO_LARGE";
    _body  = "<h1>413 REQUEST_ENTITY_TOO_LARGE</h1>";
    break;

  case HTTP_REQUEST_URI_TOO_LONG:
    _title = "414 REQUEST_URI_TOO_LONG";
    _body  = "<h1>414 REQUEST_URI_TOO_LONG</h1>";
    break;

  case HTTP_UNSUPPORTED_MEDIA_TYPE:
    _title = "415 UNSUPPORTED_MEDIA_TYPE";
    _body  = "<h1>415 UNSUPPORTED_MEDIA_TYPE</h1>";
    break;

  case HTTP_REQUESTED_RANGE_NOT_SATISFIABLE:
    _title = "416 REQUESTED_RANGE_NOT_SATISFIABLE";
    _body  = "<h1>416 REQUESTED_RANGE_NOT_SATISFIABLE</h1>";
    break;

  case HTTP_EXPECTATION_FAILED:
    _title = "417 EXPECTATION_FAILED";
    _body  = "<h1>417 EXPECTATION_FAILED</h1>";
    break;

  case HTTP_UNPROCESSABLE_ENTITY:
    _title = "422 UNPROCESSABLE_ENTITY";
    _body  = "<h1>422 UNPROCESSABLE_ENTITY</h1>";
    break;

  case HTTP_LOCKED:
    _title = "423 LOCKED";
    _body  = "<h1>423 LOCKED</h1>";
    break;

  case HTTP_FAILED_DEPENDENCY:
    _title = "424 FAILED_DEPENDENCY";
    _body  = "<h1>424 FAILED_DEPENDENCY</h1>";
    break;

  case HTTP_UNORDERED_COLLECTION:
    _title = "425 UNORDERED_COLLECTION";
    _body  = "<h1>425 UNORDERED_COLLECTION</h1>";
    break;

  case HTTP_UPGRADE_REQUIRED:
    _title = "426 UPGRADE_REQUIRED";
    _body  = "<h1>426 UPGRADE_REQUIRED</h1>";
    break;

  case HTTP_RETRY_WITH:
    _title = "449 RETRY_WITH";
    _body  = "<h1>449 RETRY_WITH</h1>";
    break;


  case HTTP_INTERNAL_SERVER_ERROR:
    _title = "500 INTERNAL_SERVER_ERROR";
    _body  = "<h1>500 INTERNAL_SERVER_ERROR</h1>";
    break;

  case HTTP_NOT_IMPLEMENTED:
    _title = "501 NOT_IMPLEMENTED";
    _body  = "<h1>501 NOT_IMPLEMENTED</h1>";
    break;

  case HTTP_BAD_GATEWAY:
    _title = "502 BAD_GATEWAY";
    _body  = "<h1>502 BAD_GATEWAY</h1>";
    break;

  case HTTP_SERVICE_UNAVAILABLE:
    _title = "503 SERVICE_UNAVAILABLE";
    _body  = "<h1>503 SERVICE_UNAVAILABLE</h1>";
    break;

  case HTTP_GATEWAY_TIMEOUT:
    _title = "504 GATEWAY_TIMEOUT";
    _body  = "<h1>504 GATEWAY_TIMEOUT</h1>";
    break;

  case HTTP_HTTP_VERSION_NOT_SUPPORTED:
    _title = "505 HTTP_VERSION_NOT_SUPPORTED";
    _body  = "<h1>505 HTTP_VERSION_NOT_SUPPORTED</h1>";
    break;

  case HTTP_VARIANT_ALSO_NEGOTIATES:
    _title = "506 VARIANT_ALSO_NEGOTIATES";
    _body  = "<h1>506 VARIANT_ALSO_NEGOTIATES</h1>";
    break;

  case HTTP_INSUFFICIENT_STORAGE:
    _title = "507 INSUFFICIENT_STORAGE";
    _body  = "<h1>507 INSUFFICIENT_STORAGE</h1>";
    break;

  case HTTP_BANDWIDTH_LIMIT_EXCEEDED:
    _title = "509 BANDWIDTH_LIMIT_EXCEEDED";
    _body  = "<h1>509 BANDWIDTH_LIMIT_EXCEEDED</h1>";
    break;

  case HTTP_NOT_EXTENDED:
    _title = "510 NOT_EXTENDED";
    _body  = "<h1>510 NOT_EXTENDED</h1>";
    break;

  default:
    _title = "Unknown Error";
    _body  = "<h1>Unknown Error</h1>";
    break;
  }

  if (format) {
    va_list args;
    va_start(args, format);
    char *tmp;
    if (vasprintf(&tmp, format, args) != -1) {
      _body += std::string("<br />\n<b>") + tmp + "</b>\n";
      free(tmp);
    }
  }
}

} // end namespace fawkes
