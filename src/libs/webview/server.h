
/***************************************************************************
 *  server.h - Web server encapsulation around libmicrohttpd
 *
 *  Created: Sun Aug 30 17:38:37 2009
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

#ifndef __LIBS_WEBVIEW_SERVER_H_
#define __LIBS_WEBVIEW_SERVER_H_

#include <sys/types.h>

struct MHD_Daemon;

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Logger;
class WebRequestDispatcher;
class WebUserVerifier;

class WebServer {
 public:
  WebServer(unsigned short int port, WebRequestDispatcher *dispatcher,
	    fawkes::Logger *logger = 0);
  WebServer(unsigned short int port, WebRequestDispatcher *dispatcher,
	    const char *key_pem_filepath, const char *cert_pem_filepath,
	    fawkes::Logger *logger = 0);
  ~WebServer();

  void process();

  void setup_basic_auth(const char *realm, WebUserVerifier *verifier);

 private:
  static char * read_file(const char *filename);

 private:
  struct MHD_Daemon    *__daemon;
  WebRequestDispatcher *__dispatcher;
  fawkes::Logger       *__logger;

  unsigned short int    __port;

  char                 *__ssl_key_mem;
  char                 *__ssl_cert_mem;
};

} // end namespace fawkes

#endif
