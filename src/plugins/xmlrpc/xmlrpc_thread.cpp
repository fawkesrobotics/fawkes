
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
#include <webview/url_manager.h>
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
  try {
    __custom_server = config->get_bool("/xmlrpc/custom_server");
  } catch (Exception &e) {
    __custom_server = false;
  }
  if (__custom_server) {
    __cfg_port = config->get_uint("/xmlrpc/port");
  }

  __cache_logger.clear();

  __processor   = new XmlRpcRequestProcessor(logger);

  xmlrpc_c::registry *registry = __processor->registry();
  __plugin_methods = new XmlRpcPluginMethods(registry, plugin_manager, logger);
  __log_methods    = new XmlRpcLogMethods(registry, &__cache_logger, logger);

  if (__custom_server) {
    __url_manager = new WebUrlManager();
    __dispatcher  = new WebRequestDispatcher(__url_manager);
    __webserver   = new WebServer(__cfg_port, __dispatcher);

    logger->log_info("XmlRpcThread", "Listening for HTTP connections on port %u",
		     __cfg_port);

    __url_manager->add_handler(WebRequest::METHOD_POST, "/",
                               std::bind(&XmlRpcRequestProcessor::process_request, __processor,
                                         std::placeholders::_1));

    __xmlrpc_service = new NetworkService(nnresolver, "Fawkes XML-RPC on %h",
					  "_http._tcp", __cfg_port);
    __xmlrpc_service->add_txt("fawkesver=%u.%u.%u", FAWKES_VERSION_MAJOR,
			      FAWKES_VERSION_MINOR, FAWKES_VERSION_MICRO);
    service_publisher->publish_service(__xmlrpc_service);
  } else {
    set_opmode(Thread::OPMODE_WAITFORWAKEUP);
    logger->log_info("XmlRpcThread", "Registering as /xmlrpc in webview");
    webview_url_manager->add_handler(WebRequest::METHOD_POST, "/xmlrpc",
                                     std::bind(&XmlRpcRequestProcessor::process_request, __processor,
                                               std::placeholders::_1));
  }

}

void
XmlRpcThread::finalize()
{
  if (__custom_server) {
    service_publisher->unpublish_service(__xmlrpc_service);
    delete __xmlrpc_service;

    delete __webserver;
    delete __plugin_methods;
    delete __dispatcher;
    delete __url_manager;
  } else {
	  webview_url_manager->remove_handler(WebRequest::METHOD_POST, "/xmlrpc");
  }
  delete __processor;
}


void
XmlRpcThread::loop()
{
  if (__custom_server) {
    __webserver->process();
  } else {
    
  }
}
