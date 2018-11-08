
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

#ifndef _FIREVISION_FVUTILS_NET_FUSE_CLIENT_H_
#define _FIREVISION_FVUTILS_NET_FUSE_CLIENT_H_

#include <fvutils/net/fuse.h>
#include <core/threading/thread.h>
#include <sys/types.h>

namespace fawkes {
  class StreamSocket;
  class WaitCondition;
  class Mutex;
}
namespace firevision {

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

  char *hostname_;
  unsigned short int port_;

  fawkes::StreamSocket *socket_;
  unsigned int wait_timeout_;

  fawkes::Mutex         *mutex_;
  fawkes::Mutex         *recv_mutex_;
  fawkes::WaitCondition *recv_waitcond_;

  FuseNetworkMessageQueue *  inbound_msgq_;
  FuseNetworkMessageQueue *  outbound_msgq_;

  FuseClientHandler       *handler_;

  bool greeting_received_;
  fawkes::Mutex         *greeting_mutex_;
  fawkes::WaitCondition *greeting_waitcond_;

  bool alive_;
};

} // end namespace firevision

#endif
