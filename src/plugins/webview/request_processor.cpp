
/***************************************************************************
 *  request_processor.cpp - HTTP request processor
 *
 *  Created: Mon Oct 13 22:02:25 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include "request_processor.h"

#include <sys/types.h>
#include <stdint.h>
#include <cstdarg>
#include <microhttpd.h>
#include <cstring>

/** @class WebRequestProcessor "request_processor.h"
 * Abstract web request processor.
 * Interface used to define web request processor that can be registered to
 * the WebRequestDispatcher.
 * @author Tim Niemueller
 *
 *
 * @fn virtual WebReply * WebRequestProcessor::process_request(const char *url, const char *method, const char *version,  const char *upload_data, size_t *upload_data_size, void **session_data) =  0
 * Process a request.
 * @param url URL, may contain escape sequences
 * @param method HTTP method
 * @param version HTTP version
 * @param upload_data uploaded data
 * @param upload_data_size size of upload_data parameter
 * @param session_data session data pointer
 * @return a WebReply instance, more specifically either a DynamicWebReply or a StaticWebReply
 * that is sent as reply, or NULL to cause a 404 (not found) error.
 */

/** Virtual empty destructor. */
WebRequestProcessor::~WebRequestProcessor()
{
}
