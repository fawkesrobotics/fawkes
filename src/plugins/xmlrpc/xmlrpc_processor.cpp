
/***************************************************************************
 *  xmlrpc_processor.cpp - XML-RPC processor
 *
 *  Created: Sun Aug 30 19:39:31 2009
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

#include "xmlrpc_processor.h"
#include <webview/page_reply.h>
#include <webview/error_reply.h>
#include <webview/request.h>
#include <logging/logger.h>

#include <xmlrpc-c/registry.hpp>
#include <cstring>

using namespace fawkes;

// accept up to 512KB as request
#define MAX_REQUEST_LENGTH (1024*512)

/** @class XmlRpcRequestProcessor "xmlrpc_processor.h"
 * XML-RPC web request processor.
 * Process web requests and pass them to the XML-RPC processor.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger logger to report problems
 */
XmlRpcRequestProcessor::XmlRpcRequestProcessor(fawkes::Logger *logger)
{
  __logger = logger;
  __xmlrpc_registry = new xmlrpc_c::registry();
}


/** Destructor. */
XmlRpcRequestProcessor::~XmlRpcRequestProcessor()
{
  delete __xmlrpc_registry;
}

/** Get XML-RPC registry.
 * @return XML-RPC registry
 */
xmlrpc_c::registry *
XmlRpcRequestProcessor::registry()
{
  return __xmlrpc_registry;
}


/** Process request.
 * @param request incoming request
 * @return web reply
 */
WebReply *
XmlRpcRequestProcessor::process_request(const fawkes::WebRequest *request)
{
  if (request->method() != WebRequest::METHOD_POST) {
    return new WebErrorPageReply(WebErrorPageReply::HTTP_METHOD_NOT_ALLOWED);
  } else {
    std::string response = "";
    __xmlrpc_registry->processCall(request->body(), &response);
    //__logger->log_debug("XmlRpcRequestProcessor", "Call: %s  reponse: %s",
    //		          request->raw_post_data().c_str(), response.c_str());
    return new StaticWebReply(WebReply::HTTP_OK, response);
  }
}
