
/***************************************************************************
 *  acceptor_thread.h - Thread accepting network connections
 *
 *  Created: Fri Nov 17 13:57:14 2006
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __NETCOMM_UTILS_ACCEPTOR_THREAD_H_
#define __NETCOMM_UTILS_ACCEPTOR_THREAD_H_

#include <core/threading/thread.h>

namespace fawkes {

class StreamSocket;
class NetworkIncomingConnectionHandler;

class NetworkAcceptorThread : public Thread
{
 public:
  NetworkAcceptorThread(NetworkIncomingConnectionHandler *handler,
			unsigned short int port,
			const char *thread_name = "NetworkAcceptorThread");
  NetworkAcceptorThread(NetworkIncomingConnectionHandler *handler,
			StreamSocket *socket,
			const char *thread_name = "NetworkAcceptorThread");
  ~NetworkAcceptorThread();

  virtual void loop();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  unsigned short int        __port;
  StreamSocket             *__socket;

  NetworkIncomingConnectionHandler *__handler;

};

} // end namespace fawkes

#endif
