
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
  handler_ = handler;
  port_    = port;

  set_prepfin_conc_loop(true);

  try {
    socket_ = new StreamSocket();
    socket_->bind(port_);
    socket_->listen();
  } catch (SocketException &e) {
    throw;
  }
}

	/** Constructor.
 * @param handler Connection handler for newly accepted incoming connections.
 * @param addr_type Specify IPv4 or IPv6
 * @param listen_addr IP address to listen on (format depends on addr_type), nullptr to listen
 * on any local (address type specific) address, e.g., :: for IPv6.
 * @param port port to listen on for incoming connections
 * @param thread_name name of the thread
 * @exception SocketException as thrown by StreamSocket connstructor, bind and listen.
 */
NetworkAcceptorThread::NetworkAcceptorThread(NetworkIncomingConnectionHandler *handler,
                                             Socket::AddrType addr_type,
                                             const std::string &listen_addr,
                                             unsigned short int port,
                                             const char *thread_name)
  : Thread(thread_name)
{
  handler_ = handler;
  port_    = port;

  set_prepfin_conc_loop(true);

  try {
    socket_ = new StreamSocket(addr_type);
    if (listen_addr.empty()) {
	    socket_->bind(port_);
    } else {
	    socket_->bind(port_, listen_addr.c_str());
    }
    socket_->listen();
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
  handler_ = handler;
  port_    = 0;
  socket_  = socket;

  set_prepfin_conc_loop(true);

  try {
    socket_->listen();
  } catch (SocketException &e) {
    throw;
  }
}


/** Destructor. */
NetworkAcceptorThread::~NetworkAcceptorThread()
{
  delete socket_;
}


/** Thread loop.
 * Waits on a socket for an incoming connection (blocking accept). If a new
 * connection has been established it is reported to the handler.
 */
void
NetworkAcceptorThread::loop()
{
  StreamSocket *s = socket_->accept<StreamSocket>();
  handler_->add_connection(s);
}

} // end namespace fawkes
