
/***************************************************************************
 *  interface_proxy.h - BlackBoard interface proxy for RemoteBlackBoard
 *
 *  Created: Tue Mar 04 10:52:28 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __BLACKBOARD_INTERFACE_PROXY_H
#define __BLACKBOARD_INTERFACE_PROXY_H_

#include <interface/mediators/interface_mediator.h>
#include <interface/mediators/message_mediator.h>
#include <cstdlib>

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

  unsigned int serial();

  /* InterfaceMediator */
  virtual bool exists_writer(const Interface *interface) const;
  virtual unsigned int num_readers(const Interface *interface) const;
  virtual void notify_of_data_change(const Interface *interface);

  /* MessageMediator */
  virtual unsigned int transmit(Message *message);

 private:
  FawkesNetworkClient *__fnc;

  RefCountRWLock      *__rwlock;
  BlackBoardNotifier  *__notifier;
  Interface           *__interface;

  void                *__mem_chunk;
  void                *__data_chunk;
  size_t               __data_size;

  unsigned int         __instance_serial;
  unsigned int         __num_readers;
  bool                 __has_writer;
  unsigned int         __clid;
};

#endif
