
/***************************************************************************
 *  connection_dispatcher.h - Network connection listener and dispatcher
 *
 *  Created: Mon Oct 20 15:02:47 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __LIBS_GUI_UTILS_CONNECTION_DISPATCHER_H_
#define __LIBS_GUI_UTILS_CONNECTION_DISPATCHER_H_

#include <cstddef>
#include <glibmm/dispatcher.h>
#include <netcomm/fawkes/client_handler.h>
#include <netcomm/fawkes/component_ids.h>
#include <core/utils/lock_queue.h>

namespace fawkes {
class FawkesNetworkClient;
class FawkesNetworkMessage;

class ConnectionDispatcher
: public FawkesNetworkClientHandler
{
 public:
  ConnectionDispatcher(unsigned int cid = FAWKES_CID_OBSERVER_MODE);
  ConnectionDispatcher(const char *hostname, unsigned short int port,
		       unsigned int cid = FAWKES_CID_OBSERVER_MODE);
  virtual ~ConnectionDispatcher();

  void set_cid(unsigned int cid);
  void set_client(FawkesNetworkClient *client);
  FawkesNetworkClient *   get_client();

  sigc::signal<void>                         signal_connected();
  sigc::signal<void>                         signal_disconnected();
  sigc::signal<void, FawkesNetworkMessage *> signal_message_received();

  virtual void deregistered(unsigned int id) throw();
  virtual void inbound_received(FawkesNetworkMessage *m, unsigned int id) throw();
  virtual void connection_died(unsigned int id) throw();
  virtual void connection_established(unsigned int id) throw();

  operator bool();

 protected:
  virtual void on_connection_established();
  virtual void on_connection_died();
  virtual void on_message_received();

 private:
  void connect_signals();

 private:
  unsigned int                                   __cid;
  FawkesNetworkClient                           *__client;
  bool                                           __client_owned;

  Glib::Dispatcher                               __dispatcher_connected;
  Glib::Dispatcher                               __dispatcher_disconnected;
  Glib::Dispatcher                               __dispatcher_message_received;

  sigc::signal<void>                             __signal_connected;
  sigc::signal<void>                             __signal_disconnected;
  sigc::signal<void, FawkesNetworkMessage *>     __signal_message_received;

  LockQueue<FawkesNetworkMessage *>              __queue_message_received;
};

}

#endif
