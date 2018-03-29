
/***************************************************************************
 *  server.h - Web server encapsulation around libmicrohttpd
 *
 *  Created: Sun Aug 30 17:38:37 2009
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

#ifndef __LIBS_WEBVIEW_SERVER_H_
#define __LIBS_WEBVIEW_SERVER_H_

#include <sys/types.h>
#include <memory>

struct MHD_Daemon;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class Time;
class WebRequestDispatcher;
class WebUserVerifier;
class WebRequestManager;

#define WEBVIEW_DEFAULT_CIPHERS "SECURE128:-VERS-SSL3.0:-VERS-TLS-ALL:+VERS-TLS1.2"

class WebServer {
 public:
  WebServer(unsigned short int port, WebRequestDispatcher *dispatcher,
            fawkes::Logger *logger = 0);
  ~WebServer();

  WebServer &  setup_tls(const char *key_pem_filepath, const char *cert_pem_filepath,
                         const char *cipher_suite = WEBVIEW_DEFAULT_CIPHERS);
  WebServer &  setup_ipv(bool enable_ipv4, bool enable_ipv6);
  WebServer &  setup_thread_pool(unsigned int num_threads);
  
  WebServer &  setup_basic_auth(const char *realm, WebUserVerifier *verifier);
  WebServer &  setup_request_manager(WebRequestManager *request_manager);
  WebServer &  setup_access_log(const char *filename);

  void start();
  void process();

  unsigned int active_requests() const;
  Time last_request_completion_time() const;

 private:
  std::string read_file(const char *filename);

 private:
  struct MHD_Daemon    *__daemon;
  WebRequestDispatcher *__dispatcher;
  WebRequestManager    *__request_manager;
  fawkes::Logger       *__logger;

  unsigned short int    __port;

  bool                  __tls_enabled;
  std::string           __tls_key_mem;
  std::string           __tls_cert_mem;
  std::string           __tls_cipher_suite;

  bool                  __enable_ipv4;
  bool                  __enable_ipv6;
  unsigned int          __num_threads;
};

} // end namespace fawkes

#endif
