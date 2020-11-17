
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

#ifndef _BLACKBOARD_INTERFACE_PROXY_H_
#define _BLACKBOARD_INTERFACE_PROXY_H_

#include <interface/mediators/interface_mediator.h>
#include <interface/mediators/message_mediator.h>
#include <utils/uuid.h>

#include <cstdlib>

namespace fawkes {

class FawkesNetworkClient;
class FawkesNetworkMessage;
class RefCountRWLock;
class BlackBoardNotifier;
class Interface;

class BlackBoardInterfaceProxy : public InterfaceMediator, public MessageMediator
{
public:
	BlackBoardInterfaceProxy(FawkesNetworkClient * client,
	                         FawkesNetworkMessage *msg,
	                         BlackBoardNotifier *  notifier,
	                         Interface *           interface,
	                         bool                  readwrite);
	~BlackBoardInterfaceProxy();

	void process_data_refreshed(FawkesNetworkMessage *msg);
	void process_interface_message(FawkesNetworkMessage *msg);
	void reader_added(Uuid event_serial);
	void reader_removed(Uuid event_serial);
	void writer_added(Uuid event_serial);
	void writer_removed(Uuid event_serial);

	Uuid       serial() const;
	Uuid       clid() const;
	Interface *interface() const;

	/* InterfaceMediator */
	virtual bool         exists_writer(const Interface *interface) const;
	virtual unsigned int num_readers(const Interface *interface) const;
	virtual void         notify_of_data_refresh(const Interface *interface, bool has_changed);
	virtual std::list<std::string> readers(const Interface *interface) const;
	virtual std::string            writer(const Interface *interface) const;

	/* MessageMediator */
	virtual void transmit(Message *message);

private:
	inline unsigned int
	next_msg_id()
	{
		// TODO use instance_serial_
		//return ((instance_serial_ << 16) | next_msg_id_++);
		return next_msg_id_++;
	}

private:
	FawkesNetworkClient *fnc_;

	RefCountRWLock *    rwlock_;
	BlackBoardNotifier *notifier_;
	Interface *         interface_;

	void * mem_chunk_;
	void * data_chunk_;
	size_t data_size_;

	Uuid           instance_serial_;
	unsigned short next_msg_id_;
	unsigned int   num_readers_;
	bool           has_writer_;
	unsigned int   clid_;
};

} // end namespace fawkes

#endif
