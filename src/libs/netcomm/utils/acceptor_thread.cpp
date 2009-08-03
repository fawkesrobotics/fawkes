
/***************************************************************************
 *  acceptor_thread.cpp - Thread accepting Fawkes network connections
 *
 *  Created: Fri Nov 17 14:09:38 2006
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

#include <netcomm/utils/acceptor_thread.h>
#include <netcomm/utils/incoming_connection_handler.h>
#include <netcomm/socket/stream.h>

namespace fawkes {

/** @class NetworkAcceptorThread <netcomm/utils/acceptor_thread.h>
 * Network Acceptor Thread.
 * Opens and maintains a server socket and waits for incoming connections. If
 * that happens NetworkConnectionHandler::add_connection() is called.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param handler Connection handler for newly accepted incoming connections.
 * @param port port to listen on for incoming connections
 * @param thread_name name of the thread
 * @exception SocketException as thrown by StreamSocket connstructor, bind and listen.
 */
NetworkAcceptorThread::NetworkAcceptorThread(NetworkIncomingConnectionHandler *handler,
					     unsigned short int port,
					     const char *thread_name)
  : Thread(thread_name)
{
  __handler = handler;
  __port    = port;

  set_prepfin_conc_loop(true);

  try {
    __socket = new StreamSocket();
    __socket->bind(__port);
    __socket->listen();
  } catch (SocketException &e) {
    throw;
  }
}


/** Constructor.
 * @param handler Connection handler for newly accepted incoming connections.
 * @param socket socket, must already be bound to the desired port. Socket::listen()
 * will be called by the acceptor thread.
 * @param thread_name name of the thread
 * @exception SocketException as thrown by StreamSocket connstructor, bind and listen.
 */
NetworkAcceptorThread::NetworkAcceptorThread(NetworkIncomingConnectionHandler *handler,
					     StreamSocket *socket,
					     const char *thread_name)
  : Thread(thread_name)
{
  __handler = handler;
  __port    = 0;
  __socket  = socket;

  set_prepfin_conc_loop(true);

  try {
    __socket->listen();
  } catch (SocketException &e) {
    throw;
  }
}


/** Destructor. */
NetworkAcceptorThread::~NetworkAcceptorThread()
{
  delete __socket;
}


/** Thread loop.
 * Waits on a socket for an incoming connection (blocking accept). If a new
 * connection has been established it is reported to the handler.
 */
void
NetworkAcceptorThread::loop()
{
  StreamSocket *s = __socket->accept<StreamSocket>();
  __handler->add_connection(s);
}

} // end namespace fawkes
