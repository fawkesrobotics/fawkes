
/***************************************************************************
 *  fuse_client.h - FUSE network transport client
 *
 *  Created: Mon Jan 09 15:33:59 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_NET_FUSE_CLIENT_H_
#define __FIREVISION_FVUTILS_NET_FUSE_CLIENT_H_

#include <fvutils/net/fuse.h>
#include <core/threading/thread.h>
#include <sys/types.h>

namespace fawkes {
  class StreamSocket;
  class WaitCondition;
  class Mutex;
}
namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class FuseNetworkMessageQueue;
class FuseNetworkMessage;
class FuseClientHandler;

class FuseClient : public fawkes::Thread {
 public:
  FuseClient(const char *hostname, unsigned short int port,
	     FuseClientHandler *handler);
  virtual ~FuseClient();

  void connect();
  void disconnect();

  void enqueue(FuseNetworkMessage *m);
  void enqueue(FUSE_message_type_t type, void *payload, size_t payload_size);
  void enqueue(FUSE_message_type_t type);
  void enqueue_and_wait(FuseNetworkMessage *message);
  void enqueue_and_wait(FUSE_message_type_t type, void *payload, size_t payload_size);
  void enqueue_and_wait(FUSE_message_type_t type);
  void wait();
  void wait_greeting();

  virtual void loop();

 private:
  void send();
  void recv();
  void sleep();

  char *__hostname;
  unsigned short int __port;

  fawkes::StreamSocket *__socket;
  unsigned int __wait_timeout;

  fawkes::Mutex         *__mutex;
  fawkes::Mutex         *__recv_mutex;
  fawkes::WaitCondition *__recv_waitcond;

  FuseNetworkMessageQueue *  __inbound_msgq;
  FuseNetworkMessageQueue *  __outbound_msgq;

  FuseClientHandler       *__handler;

  bool __greeting_received;
  fawkes::Mutex         *__greeting_mutex;
  fawkes::WaitCondition *__greeting_waitcond;

  bool __alive;
};

} // end namespace firevision

#endif
