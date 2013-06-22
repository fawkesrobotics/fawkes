
/***************************************************************************
 *  server.cpp - Web server encapsulation around libmicrohttpd
 *
 *  Created: Sun Aug 30 17:40:54 2009
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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
#include <webview/request.h>
#include <core/exception.h>
#include <core/exceptions/system.h>
#include <logging/logger.h>

#include <sys/socket.h>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <microhttpd.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/// @cond INTERNALS
static void
request_completed_callback(void *cls, struct MHD_Connection *connection, void **con_cls,
			   enum MHD_RequestTerminationCode toe)
{
  WebRequest *request = static_cast<WebRequest *>(*con_cls);
  delete request;
}
/// @endcond


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
  __port         = port;
  __dispatcher   = dispatcher;
  __logger       = logger;

  __ssl_key_mem  = NULL;
  __ssl_cert_mem = NULL;

  __daemon = MHD_start_daemon(MHD_NO_FLAG,
			      __port,
			      NULL,
			      NULL,
			      WebRequestDispatcher::process_request_cb,
			      (void *)__dispatcher,
			      MHD_OPTION_NOTIFY_COMPLETED, &request_completed_callback, NULL,
			      MHD_OPTION_END);

  if ( __daemon == NULL ) {
    throw fawkes::Exception("Could not start microhttpd");
  }

}

/** SSL constructor.
 * @param port TCP port to listen on
 * @param dispatcher dispatcher to call for requests
 * @param key_pem_filepath path to PEM formatted file containing the key
 * @param cert_pem_filepath path to PEM formatted file containing the certificate
 * @param logger optional logger, used to output possible run-time problems
 */
WebServer::WebServer(unsigned short int port, WebRequestDispatcher *dispatcher,
		     const char *key_pem_filepath, const char *cert_pem_filepath,
		     fawkes::Logger *logger)
{
  __port       = port;
  __dispatcher = dispatcher;
  __logger     = logger;

  __ssl_key_mem  = read_file(key_pem_filepath);
  __ssl_cert_mem = read_file(cert_pem_filepath);

  __daemon = MHD_start_daemon(MHD_USE_SSL,
			      __port,
			      NULL,
			      NULL,
			      WebRequestDispatcher::process_request_cb,
			      (void *)__dispatcher,
			      MHD_OPTION_NOTIFY_COMPLETED, &request_completed_callback, NULL,
			      MHD_OPTION_HTTPS_MEM_KEY,  __ssl_key_mem,
			      MHD_OPTION_HTTPS_MEM_CERT, __ssl_cert_mem,
			      MHD_OPTION_END);

  if ( __daemon == NULL ) {
    throw fawkes::Exception("Could not start microhttpd (SSL)");
  }

}


/** Destructor. */
WebServer::~WebServer()
{
  MHD_stop_daemon(__daemon);
  __daemon = NULL;
  __dispatcher = NULL;

  if (__ssl_key_mem)   free(__ssl_key_mem);
  if (__ssl_cert_mem)  free(__ssl_cert_mem);
}


/** Read file into memory.
 * @param filename file path
 * @return memory location of file content, free after done
 */
char *
WebServer::read_file(const char *filename)
{
  FILE *f = fopen(filename, "rb");
  if (! f) {
    throw CouldNotOpenFileException(filename, errno);
  }

  long size = 0;
  if ((fseek(f, 0, SEEK_END) != 0) || ((size = ftell(f)) == 1)) {
    fclose(f);
    throw Exception("Cannot determine file size of %s", filename);
  }
  fseek(f, 0, SEEK_SET);

  if ( size == 0 ) {
    fclose(f);
    throw Exception("File %s has zero length", filename);
  } else if (size > 1024 * 1024) {
    // keys or certs should not be that long...
    fclose(f);
    throw Exception("File %s is unexpectedly large", filename);
  }

  char *rv = (char *)malloc(size);
  if (fread(rv, size, 1, f) != 1) {
    int terrno = errno;
    fclose(f);
    free(rv);
    throw FileReadException(filename, terrno);
  }

  fclose(f);

  return rv;
}


/** Setup basic authentication.
 * @param realm authentication realm to display to the user
 * @param verifier verifier to use for checking credentials
 */
void
WebServer::setup_basic_auth(const char *realm, WebUserVerifier *verifier)
{
  __dispatcher->setup_basic_auth(realm, verifier);
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
