
/***************************************************************************
 *  webview_thread.cpp - Thread that handles web interface requests
 *
 *  Created: Mon Oct 13 17:51:31 2008 (I5 Developer's Day)
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

#include "webview_thread.h"
#include "static_processor.h"
#include "blackboard_processor.h"
#include "startpage_processor.h"
#include "plugins_processor.h"
#include "service_browse_handler.h"
#include "header_generator.h"
#include "footer_generator.h"

#include <core/version.h>
#include <webview/request_dispatcher.h>
#include <webview/page_reply.h>

#include <microhttpd.h>

using namespace fawkes;


/** Prefix for the WebStaticRequestProcessor. */
const char *WebviewThread::STATIC_URL_PREFIX = "/static";
/** Prefix for the WebBlackBoardRequestProcessor. */
const char *WebviewThread::BLACKBOARD_URL_PREFIX = "/blackboard";
/** Prefix for the WebPluginsRequestProcessor. */
const char *WebviewThread::PLUGINS_URL_PREFIX = "/plugins";

/** @class WebviewThread "webview_thread.h"
 * Webview Thread.
 * This thread runs the HTTP server and handles requests via the
 * WebRequestDispatcher.
 * @author Tim Niemueller
 */


/** Constructor. */
WebviewThread::WebviewThread()
  : Thread("WebviewThread", Thread::OPMODE_CONTINUOUS),
    LoggerAspect(&__cache_logger)
{
  set_prepfin_conc_loop(true);
  
}


WebviewThread::~WebviewThread()
{
}

void
WebviewThread::init()
{
  __cfg_port = config->get_uint("/webview/port");

  __cache_logger.clear();

  __webview_service = new NetworkService(nnresolver, "Fawkes Webview on %h",
					 "_http._tcp", __cfg_port);
  __webview_service->add_txt("fawkesver=%u.%u.%u",
			     FAWKES_VERSION_MAJOR, FAWKES_VERSION_MINOR,
			     FAWKES_VERSION_MICRO);
  __service_browse_handler = new WebviewServiceBrowseHandler(logger, __webview_service);

  __header_gen = new WebviewHeaderGenerator();
  __footer_gen = new WebviewFooterGenerator(__service_browse_handler);

  __dispatcher = new WebRequestDispatcher(__header_gen, __footer_gen);
  __daemon = MHD_start_daemon(MHD_NO_FLAG,
			      __cfg_port,
			      NULL,
			      NULL,
			      WebRequestDispatcher::process_request_cb,
			      (void *)__dispatcher,
			      MHD_OPTION_END);

  if ( __daemon == NULL ) {
    throw Exception("Could not start microhttpd");
  }

  __startpage_processor  = new WebviewStartPageRequestProcessor(&__cache_logger);
  __static_processor     = new WebviewStaticRequestProcessor(STATIC_URL_PREFIX, RESDIR"/webview", logger);
  __blackboard_processor = new WebviewBlackBoardRequestProcessor(BLACKBOARD_URL_PREFIX, blackboard);
  __plugins_processor    = new WebviewPluginsRequestProcessor(PLUGINS_URL_PREFIX, plugin_manager);
  __dispatcher->add_processor("/", __startpage_processor);
  __dispatcher->add_processor(STATIC_URL_PREFIX, __static_processor);
  __dispatcher->add_processor(BLACKBOARD_URL_PREFIX, __blackboard_processor);
  __dispatcher->add_processor(PLUGINS_URL_PREFIX, __plugins_processor);

  __header_gen->add_nav_entry(BLACKBOARD_URL_PREFIX, "BlackBoard");
  __header_gen->add_nav_entry(PLUGINS_URL_PREFIX, "Plugins");

  logger->log_info("WebviewThread", "Listening for HTTP connections on port %u", __cfg_port);

  service_publisher->publish_service(__webview_service);
  service_browser->watch_service("_http._tcp", __service_browse_handler);

}

void
WebviewThread::finalize()
{
  service_publisher->unpublish_service(__webview_service);
  service_browser->unwatch_service("_http._tcp", __service_browse_handler);

  MHD_stop_daemon(__daemon);

  delete __webview_service;
  delete __service_browse_handler;

  delete __dispatcher;
  delete __static_processor;
  delete __blackboard_processor;
  delete __startpage_processor;
  delete __plugins_processor;
  delete __footer_gen;
  delete __header_gen;
  __daemon = NULL;
  __dispatcher = NULL;
}


void
WebviewThread::loop()
{
  fd_set read_fd, write_fd, except_fd;
  int max_fd = 0;
  FD_ZERO(&read_fd); FD_ZERO(&write_fd); FD_ZERO(&except_fd);
  if ( MHD_get_fdset(__daemon, &read_fd, &write_fd, &except_fd, &max_fd) != MHD_YES ) {
    logger->log_warn("WebviewThread", "Could not get microhttpd fdsets");
    return;
  }
  select(max_fd + 1, &read_fd, &write_fd, &except_fd, NULL);
  MHD_run(__daemon);
}
