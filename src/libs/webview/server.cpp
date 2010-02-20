
/***************************************************************************
 *  server.cpp - Web server encapsulation around libmicrohttpd
 *
 *  Created: Sun Aug 30 17:40:54 2009
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

#include <webview/server.h>
#include <webview/request_dispatcher.h>
#include <core/exception.h>
#include <utils/logging/logger.h>

#include <sys/socket.h>
#include <microhttpd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class WebServer <webview/server.h>
 * Encapsulation of the libmicrohttpd webserver.
 * This class opens a port serving websites and calls the supplied dispatcher
 * for requests.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param port TCP port to listen on
 * @param dispatcher dispatcher to call for requests
 * @param logger optional logger, used to output possible run-time problems
 */
WebServer::WebServer(unsigned short int port, WebRequestDispatcher *dispatcher,
		     fawkes::Logger *logger)
{
  __port       = port;
  __dispatcher = dispatcher;
  __logger     = logger;

  __daemon = MHD_start_daemon(MHD_NO_FLAG,
			      __port,
			      NULL,
			      NULL,
			      WebRequestDispatcher::process_request_cb,
			      (void *)__dispatcher,
			      MHD_OPTION_END);

  if ( __daemon == NULL ) {
    throw fawkes::Exception("Could not start microhttpd");
  }

}


/** Destructor. */
WebServer::~WebServer()
{
  MHD_stop_daemon(__daemon);
  __daemon = NULL;
  __dispatcher = NULL;
}

/** Process requests.
 * This method waits for new requests and processes them when received.
 */
void
WebServer::process()
{
  fd_set read_fd, write_fd, except_fd;
  int max_fd = 0;
  FD_ZERO(&read_fd); FD_ZERO(&write_fd); FD_ZERO(&except_fd);
  if ( MHD_get_fdset(__daemon, &read_fd, &write_fd, &except_fd, &max_fd) != MHD_YES ) {
    if (__logger)
      __logger->log_warn("WebviewThread", "Could not get microhttpd fdsets");
    return;
  }
  select(max_fd + 1, &read_fd, &write_fd, &except_fd, NULL);
  MHD_run(__daemon);
}

} // end namespace fawkes
