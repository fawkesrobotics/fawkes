
/***************************************************************************
 *  server.cpp - Web server encapsulation around libmicrohttpd
 *
 *  Created: Sun Aug 30 17:40:54 2009
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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
#include <webview/request_manager.h>
#include <webview/access_log.h>
#include <core/threading/thread.h>
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
WebServer::WebServer(unsigned short int port,
                     WebRequestDispatcher *dispatcher,
                     fawkes::Logger *logger)
{
  port_         = port;
  dispatcher_   = dispatcher;
  logger_       = logger;
  request_manager_ = NULL;

  enable_ipv4_ = true;
  enable_ipv6_ = true;

  tls_enabled_ = false;
  num_threads_ = 1;
}

/** Setup Transport Layer Security (encryption),
 * @param key_pem_filepath path to PEM formatted file containing the key
 * @param cert_pem_filepath path to PEM formatted file containing the certificate
 * @param cipher_suite which cipers to use for SSL/TLS connections
 * @return *this to allow for chaining
 */
WebServer &
WebServer::setup_tls(const char *key_pem_filepath, const char *cert_pem_filepath,
                     const char *cipher_suite)
{
	tls_enabled_  = true;
	tls_key_mem_  = std::move(read_file(key_pem_filepath));
	tls_cert_mem_ = std::move(read_file(cert_pem_filepath));
  if (cipher_suite == NULL) {
	  tls_cipher_suite_ = WEBVIEW_DEFAULT_CIPHERS;
  } else {
	  tls_cipher_suite_ = cipher_suite;
  }

  return *this;
}

/** Setup protocols, i.e., IPv4 and/or IPv6.
 * @param enable_ipv4 enable IPv4 support
 * @param enable_ipv6 enable IPv6 support
 * @return *this to allow for chaining
 */
WebServer &
WebServer::setup_ipv(bool enable_ipv4, bool enable_ipv6)
{
	enable_ipv4_ = enable_ipv4;
	enable_ipv6_ = enable_ipv6;

  return *this;
}

/** Setup cross-origin resource sharing
 * @param allow_all allow access to all hosts
 * @param origins allow access from these specific origins
 * @param max_age maximum cache time to send to the client, zero to disable
 * @return *this to allow for chaining
 */
WebServer &
WebServer::setup_cors(bool allow_all, std::vector<std::string>&& origins,
                      unsigned int max_age)
{
	cors_allow_all_ = allow_all;
	cors_origins_   = std::move(origins);
	cors_max_age_   = max_age;

  return *this;
}

/** Setup thread pool.
 * This also enables epoll on Linux.
 * @param num_threads number of threads in thread pool. If this equals
 * one, thread pooling will be disabled and will process requests from
 * within webview thread.
 * @return *this to allow for chaining
 */
WebServer &
WebServer::setup_thread_pool(unsigned int num_threads)
{
	num_threads_ = num_threads;

  return *this;
}


/** Start daemon and enable processing requests.
 */
void
WebServer::start()
{
  unsigned int flags = MHD_NO_FLAG;
#if MHD_VERSION >= 0x00090280
  if (enable_ipv4_ && enable_ipv6_) {
	  flags |= MHD_USE_DUAL_STACK;
  } else if (enable_ipv6_) {
	  flags |= MHD_USE_IPv6;
  } else if (! enable_ipv4_ && ! enable_ipv6_) {
	  throw fawkes::Exception("WebServer: neither IPv4 nor IPv6 enabled");
  }
#endif

  if (tls_enabled_) {
	  flags |= MHD_USE_SSL;
  }

  dispatcher_->setup_cors(cors_allow_all_, std::move(cors_origins_), cors_max_age_);

  if (num_threads_ > 1) {
#ifdef __linux__
	  flags |= MHD_USE_EPOLL_LINUX_ONLY;
#endif
	  flags |= MHD_USE_SELECT_INTERNALLY;
  }

  size_t num_options = 3 + (num_threads_ > 1 ? 1 : 0) + (tls_enabled_ ? 3 : 0);

  size_t cur_op = 0;
  struct MHD_OptionItem ops[num_options];
  ops[cur_op++] = MHD_OptionItem{ MHD_OPTION_NOTIFY_COMPLETED,
                                  (intptr_t)WebRequestDispatcher::request_completed_cb,
                                  (void *)dispatcher_ };
  ops[cur_op++] = MHD_OptionItem{ MHD_OPTION_URI_LOG_CALLBACK,
                                  (intptr_t)WebRequestDispatcher::uri_log_cb,
                                  (void *)dispatcher_ };

  if (num_threads_ > 1) {
	  ops[cur_op++] = MHD_OptionItem{ MHD_OPTION_THREAD_POOL_SIZE, num_threads_, NULL };
  }

  if (tls_enabled_) {
	  ops[cur_op++] = MHD_OptionItem{ MHD_OPTION_HTTPS_MEM_KEY,
	                                  (intptr_t)tls_key_mem_.c_str(), NULL };
	  ops[cur_op++] = MHD_OptionItem{ MHD_OPTION_HTTPS_MEM_CERT,
	                                  (intptr_t)tls_cert_mem_.c_str(), NULL };
	  ops[cur_op++] = MHD_OptionItem{ MHD_OPTION_HTTPS_PRIORITIES,
	                                  (intptr_t)tls_cipher_suite_.c_str(), NULL };
  }

  ops[cur_op++] = MHD_OptionItem{ MHD_OPTION_END, 0, NULL };

  daemon_ = MHD_start_daemon(flags, port_, NULL, NULL,
                              WebRequestDispatcher::process_request_cb,
                              (void *)dispatcher_,
                              MHD_OPTION_ARRAY, ops,
                              MHD_OPTION_END);

  if ( daemon_ == NULL ) {
    throw fawkes::Exception("Could not start microhttpd");
  }
}

/** Destructor. */
WebServer::~WebServer()
{
  if (request_manager_) {
    request_manager_->set_server(NULL);
  }

  MHD_stop_daemon(daemon_);
  daemon_ = NULL;
  dispatcher_ = NULL;
}


/** Read file into memory.
 * @param filename file path
 * @return string with file content.
 * Note that this expects reasonably small file sizes that can be held
 * in memory completely, as is the case for TLS certificates.
 */
std::string
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

  std::string rv(size+1, 0);
  if (fread(&rv[0], size, 1, f) != 1) {
    int terrno = errno;
    fclose(f);
    throw FileReadException(filename, terrno);
  }
  fclose(f);

  return rv;
}


/** Setup basic authentication.
 * @param realm authentication realm to display to the user
 * @param verifier verifier to use for checking credentials
 * @return *this to allow for chaining
 */
WebServer &
WebServer::setup_basic_auth(const char *realm, WebUserVerifier *verifier)
{
  dispatcher_->setup_basic_auth(realm, verifier);
  return *this;
}


/** Setup access log.
 * @param filename access log file name
 * @return *this to allow for chaining
 */
WebServer &
WebServer::setup_access_log(const char *filename)
{
  dispatcher_->setup_access_log(filename);
  return *this;
}


/** Setup this server as request manager.
 * The registration will be cancelled automatically on destruction.
 * @param request_manager request manager to register with
 * @return *this to allow for chaining
 */
WebServer &
WebServer::setup_request_manager(WebRequestManager *request_manager)
{
  request_manager->set_server(this);
  request_manager_ = request_manager;
return *this;
}

/** Get number of active requests.
 * @return number of ongoing requests.
 */
unsigned int
WebServer::active_requests() const
{
  return dispatcher_->active_requests();
}

/** Get time when last request was completed.
 * @return Time when last request was completed
 */
Time
WebServer::last_request_completion_time() const
{
  return dispatcher_->last_request_completion_time();
}


/** Process requests.
 * This method waits for new requests and processes them when
 * received. It is necessary to call this function if running the
 * server in single thread mode, i.e., setup_thread_pool() has not
 * been called or only for a single thread.  The function may always
 * be called safely, even in thread pool mode. However, when called in
 * thread pool mode, the function will always return immediately.
 */
void
WebServer::process()
{
	if (num_threads_ > 1) {
		// nothing to be done when using thread pool mode
		return;
	}

  fd_set read_fd, write_fd, except_fd;
  int max_fd = 0;
  FD_ZERO(&read_fd); FD_ZERO(&write_fd); FD_ZERO(&except_fd);
  if ( MHD_get_fdset(daemon_, &read_fd, &write_fd, &except_fd, &max_fd) != MHD_YES ) {
    if (logger_)
      logger_->log_warn("WebviewThread", "Could not get microhttpd fdsets");
    return;
  }
  select(max_fd + 1, &read_fd, &write_fd, &except_fd, NULL);
  Thread::CancelState old_state;
  Thread::set_cancel_state(Thread::CANCEL_DISABLED, &old_state);
  MHD_run(daemon_);
  Thread::set_cancel_state(old_state);
}

} // end namespace fawkes
