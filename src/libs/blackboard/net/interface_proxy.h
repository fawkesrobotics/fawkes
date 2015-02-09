
/***************************************************************************
 *  interface_proxy.h - BlackBoard interface proxy for RemoteBlackBoard
 *
 *  Created: Tue Mar 04 10:52:28 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_INTERFACE_PROXY_H_
#define __BLACKBOARD_INTERFACE_PROXY_H_

#include <interface/mediators/interface_mediator.h>
#include <interface/mediators/message_mediator.h>
#include <cstdlib>

namespace fawkes {

class FawkesNetworkClient;
class FawkesNetworkMessage;
class RefCountRWLock;
class BlackBoardNotifier;
class Interface;

class BlackBoardInterfaceProxy
: public InterfaceMediator,
  public MessageMediator
{
 public:
  BlackBoardInterfaceProxy(FawkesNetworkClient *client, FawkesNetworkMessage *msg,
			   BlackBoardNotifier *notifier, Interface *interface,
			   bool readwrite);
  ~BlackBoardInterfaceProxy();

  void process_data_changed(FawkesNetworkMessage *msg);
  void process_interface_message(FawkesNetworkMessage *msg);
  void reader_added(unsigned int event_serial);
  void reader_removed(unsigned int event_serial);
  void writer_added(unsigned int event_serial);
  void writer_removed(unsigned int event_serial);

  unsigned int serial() const;
  unsigned int clid() const;
  Interface *  interface() const;

  /* InterfaceMediator */
  virtual bool exists_writer(const Interface *interface) const;
  virtual unsigned int num_readers(const Interface *interface) const;
  virtual void notify_of_data_change(const Interface *interface);
  virtual std::list<std::string>  readers(const Interface *interface) const;
  virtual std::string             writer(const Interface *interface) const;

  /* MessageMediator */
  virtual void transmit(Message *message);

 private:
  inline unsigned int next_msg_id()
  {
    return ((__instance_serial << 16) | __next_msg_id++);
  }

 private:
  FawkesNetworkClient *__fnc;

  RefCountRWLock      *__rwlock;
  BlackBoardNotifier  *__notifier;
  Interface           *__interface;

  void                *__mem_chunk;
  void                *__data_chunk;
  size_t               __data_size;

  unsigned short       __instance_serial;
  unsigned short       __next_msg_id;
  unsigned int         __num_readers;
  bool                 __has_writer;
  unsigned int         __clid;
};

} // end namespace fawkes

#endif
