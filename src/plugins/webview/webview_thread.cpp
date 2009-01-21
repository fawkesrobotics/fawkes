
/***************************************************************************
 *  webview_thread.cpp - Thread that handles web interface requests
 *
 *  Created: Mon Oct 13 17:51:31 2008 (I5 Developer's Day)
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

#include "webview_thread.h"
#include "request_dispatcher.h"
#include "static_processor.h"
#include "blackboard_processor.h"
#include "page_reply.h"

#include <microhttpd.h>

using namespace fawkes;


/** Prefix for the StaticRequestProcessor. */
const char *WebviewThread::STATIC_URL_PREFIX = "/static";
/** Prefix for the BlackBoardRequestProcessor. */
const char *WebviewThread::BLACKBOARD_URL_PREFIX = "/blackboard";

/** @class WebviewThread "webview_thread.h"
 * Webview Thread.
 * This thread runs the HTTP server and handles requests via the
 * WebRequestDispatcher.
 * @author Tim Niemueller
 */


/** Constructor. */
WebviewThread::WebviewThread()
  : Thread("WebviewThread", Thread::OPMODE_CONTINUOUS)
{
  set_prepfin_conc_loop(true);
}


void
WebviewThread::init()
{
  __cfg_port = config->get_uint("/webview/port");

  __dispatcher = new WebRequestDispatcher();
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

  __static_processor = new WebStaticRequestProcessor(STATIC_URL_PREFIX, RESDIR"/webview", logger);
  __blackboard_processor = new WebBlackBoardRequestProcessor(BLACKBOARD_URL_PREFIX, blackboard);
  __dispatcher->add_processor(STATIC_URL_PREFIX, __static_processor);
  __dispatcher->add_processor(BLACKBOARD_URL_PREFIX, __blackboard_processor);

  WebPageReply::add_nav_entry(BLACKBOARD_URL_PREFIX, "BlackBoard");

  logger->log_info("WebviewThread", "Listening for HTTP connections on port %u", __cfg_port);
}


void
WebviewThread::finalize()
{
  WebPageReply::remove_nav_entry(BLACKBOARD_URL_PREFIX);
  MHD_stop_daemon(__daemon);
  delete __dispatcher;
  delete __static_processor;
  delete __blackboard_processor;
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
