
/***************************************************************************
 *  acceptor_thread.cpp - Thread accepting Fawkes network connections
 *
 *  Generated: Fri Nov 17 14:09:38 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <netcomm/fawkes/acceptor_thread.h>
#include <netcomm/fawkes/client_thread.h>
#include <netcomm/fawkes/network_thread.h>
#include <netcomm/socket/stream.h>

/** @class FawkesNetworkAcceptorThread netcomm/fawkes/acceptor_thread.h
 * Fawkes Network Acceptor Thread.
 * Opens and maintains Fawkes sockets and waits for incoming connections. If
 * that happens the parents FawkesNetworkThread::add_client() is called and
 * the parent thread is woken up.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor.
 * @param parent parent network thread to add clients to
 * @param port port to listen on for incoming connections
 * @exception SocketException as thrown by StreamSocket connstructor, bind and listen.
 */
FawkesNetworkAcceptorThread::FawkesNetworkAcceptorThread(FawkesNetworkThread *parent,
							 unsigned short int port)
  : Thread("FawkesNetworkAcceptorThread")
{
  this->parent = parent;
  this->port   = port;

  try {
    socket = new StreamSocket();
    socket->bind(port);
    socket->listen();
  } catch (SocketException &e) {
    throw;
  }
}


/** Destructor. */
FawkesNetworkAcceptorThread::~FawkesNetworkAcceptorThread()
{
  delete socket;
}


/** Thread loop.
 * Waits on a socket for an incoming connection (blocking accept). If a new
 * connection has been established it is reported to the parent thread and
 * the parent thread is woken up.
 */
void
FawkesNetworkAcceptorThread::loop()
{
  StreamSocket *s = socket->accept<StreamSocket>();
  FawkesNetworkClientThread *ct = new FawkesNetworkClientThread(s, parent);
  parent->add_client(ct);
  parent->wakeup();
}
