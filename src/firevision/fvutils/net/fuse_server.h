
/***************************************************************************
 *  fuse_server.h - network image transport server interface
 *
 *  Generated: Mon Jan 09 15:26:27 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_SERVER_H_
#define __FIREVISION_FVUTILS_NET_FUSE_SERVER_H_

#include <core/threading/thread.h>
#include <core/utils/lock_list.h>
#include <netcomm/utils/incoming_connection_handler.h>

class ThreadCollector;
class StreamSocket;
class NetworkAcceptorThread;
class FuseServerClientThread;

class FuseServer : public Thread, public NetworkIncomingConnectionHandler {
 public:

  FuseServer(unsigned short int port, ThreadCollector *collector = NULL);
  virtual ~FuseServer();

  virtual void add_connection(StreamSocket *s) throw();
  void connection_died(FuseServerClientThread *client) throw();

  virtual void loop();

 private:
  NetworkAcceptorThread *__acceptor_thread;

  LockList<FuseServerClientThread *>  __clients;
  LockList<FuseServerClientThread *>::iterator  __cit;

  LockList<FuseServerClientThread *>  __dead_clients;

  ThreadCollector *__thread_collector;
};


#endif
