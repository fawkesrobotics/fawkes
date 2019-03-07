
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

#include "methods/log.h"
#include "methods/plugin.h"
#include "xmlrpc_processor.h"

#include <core/version.h>
#include <webview/request_dispatcher.h>
#include <webview/server.h>
#include <webview/url_manager.h>

using namespace fawkes;

/** @class XmlRpcThread "xmlrpc_thread.h"
 * XML-RPC Thread.
 * This thread runs the HTTP server and handles XML-RPC calls.
 * @author Tim Niemueller
 */

/** Constructor. */
XmlRpcThread::XmlRpcThread()
: Thread("XmlRpcThread", Thread::OPMODE_CONTINUOUS), LoggerAspect(&cache_logger_)
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
		custom_server_ = config->get_bool("/xmlrpc/custom_server");
	} catch (Exception &e) {
		custom_server_ = false;
	}
	if (custom_server_) {
		cfg_port_ = config->get_uint("/xmlrpc/port");
	}

	cache_logger_.clear();

	processor_ = new XmlRpcRequestProcessor(logger);

	std::shared_ptr<xmlrpc_c::registry> registry = processor_->registry();
	plugin_methods_ = new XmlRpcPluginMethods(registry, plugin_manager, logger);
	log_methods_    = new XmlRpcLogMethods(registry, &cache_logger_, logger);

	if (custom_server_) {
		url_manager_ = new WebUrlManager();
		dispatcher_  = new WebRequestDispatcher(url_manager_);
		webserver_   = new WebServer(cfg_port_, dispatcher_);

		logger->log_info("XmlRpcThread", "Listening for HTTP connections on port %u", cfg_port_);

		url_manager_->add_handler(WebRequest::METHOD_POST,
		                          "/",
		                          std::bind(&XmlRpcRequestProcessor::process_request,
		                                    processor_,
		                                    std::placeholders::_1));

		xmlrpc_service_ =
		  new NetworkService(nnresolver, "Fawkes XML-RPC on %h", "_http._tcp", cfg_port_);
		xmlrpc_service_->add_txt("fawkesver=%u.%u.%u",
		                         FAWKES_VERSION_MAJOR,
		                         FAWKES_VERSION_MINOR,
		                         FAWKES_VERSION_MICRO);
		service_publisher->publish_service(xmlrpc_service_);
	} else {
		set_opmode(Thread::OPMODE_WAITFORWAKEUP);
		logger->log_info("XmlRpcThread", "Registering as /xmlrpc in webview");
		webview_url_manager->add_handler(WebRequest::METHOD_POST,
		                                 "/xmlrpc",
		                                 std::bind(&XmlRpcRequestProcessor::process_request,
		                                           processor_,
		                                           std::placeholders::_1));
	}
}

void
XmlRpcThread::finalize()
{
	if (custom_server_) {
		service_publisher->unpublish_service(xmlrpc_service_);
		delete xmlrpc_service_;

		delete webserver_;
		delete plugin_methods_;
		delete log_methods_;
		delete dispatcher_;
		delete url_manager_;
	} else {
		webview_url_manager->remove_handler(WebRequest::METHOD_POST, "/xmlrpc");
	}
	delete processor_;
}

void
XmlRpcThread::loop()
{
	if (custom_server_) {
		webserver_->process();
	}
}
