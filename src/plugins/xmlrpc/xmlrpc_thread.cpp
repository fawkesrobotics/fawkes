
/***************************************************************************
 *  xmlrpc_thread.cpp - Thread that handles xml-rpc requests
 *
 *  Created: Sun Aug 30 12:49:26 2009
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

#include "xmlrpc_thread.h"
#include "xmlrpc_processor.h"
#include "methods/plugin.h"
#include "methods/log.h"

#include <core/version.h>
#include <webview/server.h>
#include <webview/request_dispatcher.h>

using namespace fawkes;


/** @class XmlRpcThread "xmlrpc_thread.h"
 * XML-RPC Thread.
 * This thread runs the HTTP server and handles XML-RPC calls.
 * @author Tim Niemueller
 */


/** Constructor. */
XmlRpcThread::XmlRpcThread()
  : Thread("XmlRpcThread", Thread::OPMODE_CONTINUOUS),
    LoggerAspect(&__cache_logger)
{
  set_prepfin_conc_loop(true);
  
}


XmlRpcThread::~XmlRpcThread()
{
}

void
XmlRpcThread::init()
{
  __cfg_port = config->get_uint("/xmlrpc/port");

  __cache_logger.clear();

  __dispatcher = new WebRequestDispatcher();
  __webserver  = new WebServer(__cfg_port, __dispatcher);

  __processor  = new XmlRpcRequestProcessor(logger);

  xmlrpc_c::registry *registry = __processor->registry();
  __plugin_methods = new XmlRpcPluginMethods(registry, plugin_manager, logger);
  __log_methods    = new XmlRpcLogMethods(registry, &__cache_logger, logger);

  __dispatcher->add_processor("/", __processor);

  logger->log_info("XmlRpcThread", "Listening for HTTP connections on port %u", __cfg_port);


  __xmlrpc_service = new NetworkService(nnresolver, "Fawkes XML-RPC on %h",
					"_http._tcp", __cfg_port);
  __xmlrpc_service->add_txt("fawkesver=%u.%u.%u", FAWKES_VERSION_MAJOR,
			    FAWKES_VERSION_MINOR, FAWKES_VERSION_MICRO);
  service_publisher->publish_service(__xmlrpc_service);


}

void
XmlRpcThread::finalize()
{
  service_publisher->unpublish_service(__xmlrpc_service);
  delete __xmlrpc_service;

  delete __webserver;
  delete __plugin_methods;
  delete __dispatcher;
  delete __processor;
}


void
XmlRpcThread::loop()
{
  __webserver->process();
}
