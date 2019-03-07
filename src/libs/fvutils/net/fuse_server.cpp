
/***************************************************************************
 *  fuse_server.tcp - network image transport server interface
 *
 *  Generated: Mon Mar 19 15:56:22 2007
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

#include <core/threading/thread_collector.h>
#include <fvutils/net/fuse_server.h>
#include <fvutils/net/fuse_server_client_thread.h>
#include <netcomm/utils/acceptor_thread.h>

#include <algorithm>

using namespace fawkes;

namespace firevision {

/** @class FuseServer <fvutils/net/fuse_server.h>
 * FireVision FUSE protocol server.
 * The FuseServer will open a StreamSocket and listen on it for incoming
 * connections. For each connection a client thread is started that will process
 * all requests issued by the client.
 *
 * @ingroup FUSE
 * @ingroup FireVision
 * @author Tim Niemueller
 */

/** Constructor.
 * @param enable_ipv4 true to listen on the IPv4 TCP port
 * @param enable_ipv6 true to listen on the IPv6 TCP port
 * @param listen_ipv4 IPv4 address to listen on for incoming connections,
 * 0.0.0.0 to listen on any local address
 * @param listen_ipv6 IPv6 address to listen on for incoming connections,
 * :: to listen on any local address
 * @param port Port to listen on for incoming connections
 * @param collector optional thread collector
 */
FuseServer::FuseServer(bool               enable_ipv4,
                       bool               enable_ipv6,
                       const std::string &listen_ipv4,
                       const std::string &listen_ipv6,
                       unsigned short int port,
                       ThreadCollector *  collector)
: Thread("FuseServer", Thread::OPMODE_WAITFORWAKEUP)
{
	thread_collector_ = collector;

	if (enable_ipv4) {
		acceptor_threads_.push_back(new NetworkAcceptorThread(
		  this, Socket::IPv4, listen_ipv4, port, "FuseNetworkAcceptorThread"));
	}
	if (enable_ipv6) {
		acceptor_threads_.push_back(new NetworkAcceptorThread(
		  this, Socket::IPv6, listen_ipv6, port, "FuseNetworkAcceptorThread"));
	}
	if (thread_collector_) {
		for (size_t i = 0; i < acceptor_threads_.size(); ++i) {
			thread_collector_->add(acceptor_threads_[i]);
		}
	} else {
		for (size_t i = 0; i < acceptor_threads_.size(); ++i) {
			acceptor_threads_[i]->start();
		}
	}
}

/** Destructor. */
FuseServer::~FuseServer()
{
	for (size_t i = 0; i < acceptor_threads_.size(); ++i) {
		if (thread_collector_) {
			thread_collector_->remove(acceptor_threads_[i]);
		} else {
			acceptor_threads_[i]->cancel();
			acceptor_threads_[i]->join();
		}
		delete acceptor_threads_[i];
	}
	acceptor_threads_.clear();

	for (cit_ = clients_.begin(); cit_ != clients_.end(); ++cit_) {
		if (thread_collector_) {
			// ThreadCollector::remove also stops the threads!
			thread_collector_->remove(*cit_);
		} else {
			(*cit_)->cancel();
			(*cit_)->join();
		}
		delete *cit_;
	}
	clients_.clear();
}

void
FuseServer::add_connection(StreamSocket *s) throw()
{
	FuseServerClientThread *client = new FuseServerClientThread(this, s);
	if (thread_collector_) {
		thread_collector_->add(client);
	} else {
		client->start();
	}
	clients_.push_back_locked(client);
}

/** Connection died.
 * @param client client whose connection died
 */
void
FuseServer::connection_died(FuseServerClientThread *client) throw()
{
	dead_clients_.push_back_locked(client);
	wakeup();
}

void
FuseServer::loop()
{
	// Check for dead clients, cancel and join if there are any
	dead_clients_.lock();
	clients_.lock();

	LockList<FuseServerClientThread *>::iterator dcit;

	while (!dead_clients_.empty()) {
		dcit = dead_clients_.begin();

		if (thread_collector_) {
			// ThreadCollector::remove also stops the threads!
			thread_collector_->remove(*dcit);
		} else {
			(*dcit)->cancel();
			(*dcit)->join();
		}
		if ((cit_ = find(clients_.begin(), clients_.end(), *dcit)) != clients_.end()) {
			clients_.erase(cit_);
		}

		FuseServerClientThread *tc = *dcit;
		dead_clients_.erase(dcit);
		delete tc;
	}

	clients_.unlock();
	dead_clients_.unlock();
}

} // end namespace firevision
