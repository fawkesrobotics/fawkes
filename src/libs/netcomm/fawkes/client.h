
/***************************************************************************
 *  client.h - Fawkes network client
 *
 *  Created: Tue Nov 21 18:42:10 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef _NETCOMM_FAWKES_CLIENT_H_
#define _NETCOMM_FAWKES_CLIENT_H_

#include <core/exception.h>
#include <core/utils/lock_map.h>
#include <netcomm/fawkes/component_ids.h>
#include <netcomm/fawkes/message.h>
#include <netcomm/fawkes/message_queue.h>
#include <sys/socket.h>

namespace fawkes {

class StreamSocket;
class Mutex;
class WaitCondition;
class FawkesNetworkClientHandler;
class FawkesNetworkClientSendThread;
class FawkesNetworkClientRecvThread;

class HandlerAlreadyRegisteredException : public Exception
{
public:
	HandlerAlreadyRegisteredException();
};

#define FAWKES_TCP_PORT 1910

class FawkesNetworkClient
{
	friend FawkesNetworkClientSendThread;
	friend FawkesNetworkClientRecvThread;

public:
	FawkesNetworkClient();
	FawkesNetworkClient(const char *host, unsigned short int port);
	FawkesNetworkClient(unsigned int id, const char *host, unsigned short int port);
	~FawkesNetworkClient();

	void connect();
	void disconnect();
	void connect(const char *host, unsigned short int port);
	void connect(const char *hostname, const struct sockaddr *addr, socklen_t addrlen);
	void connect(const char *hostname, const struct sockaddr_storage &addr);

	void enqueue(FawkesNetworkMessage *message);
	void enqueue_and_wait(FawkesNetworkMessage *message, unsigned int timeout_sec = 15);

	void wait(unsigned int component_id, unsigned int timeout_sec = 15);
	void wake(unsigned int component_id);

	void interrupt_connect();

	void register_handler(FawkesNetworkClientHandler *handler, unsigned int component_id);
	void deregister_handler(unsigned int component_id);

	bool connected() const throw();

	bool         has_id() const;
	unsigned int id() const;

	const char *get_hostname() const;

private:
	void recv();
	void notify_of_connection_established();
	void notify_of_connection_dead();

	void wake_handlers(unsigned int cid);
	void dispatch_message(FawkesNetworkMessage *m);
	void connection_died();
	void set_send_slave_alive();
	void set_recv_slave_alive();

	char *             host_;
	unsigned short int port_;

	StreamSocket *s;

	typedef LockMap<unsigned int, FawkesNetworkClientHandler *> HandlerMap;
	HandlerMap                                                  handlers;

	WaitCondition *connest_waitcond_;
	Mutex *        connest_mutex_;
	bool           connest_;
	bool           connest_interrupted_;

	Mutex *                        recv_mutex_;
	WaitCondition *                recv_waitcond_;
	std::map<unsigned int, bool>   recv_received_;
	FawkesNetworkClientRecvThread *recv_slave_;
	FawkesNetworkClientSendThread *send_slave_;
	bool                           recv_slave_alive_;
	bool                           send_slave_alive_;

	bool         connection_died_recently;
	Mutex *      slave_status_mutex;
	bool         _has_id;
	unsigned int _id;

	struct sockaddr *addr_;
	socklen_t        addr_len_;
};

} // end namespace fawkes

#endif
