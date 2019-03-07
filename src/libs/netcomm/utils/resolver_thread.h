
/***************************************************************************
 *  resolver_thread.h - Fawkes network name resolver thread
 *
 *  Created: Fri May 11 22:10:03 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef _NETCOMM_UTILS_RESOLVER_THREAD_H_
#define _NETCOMM_UTILS_RESOLVER_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lock_hashmap.h>
#include <core/utils/lock_hashset.h>
#include <utils/misc/string_compare.h>
#ifdef HAVE_AVAHI
#	include <netcomm/dns-sd/avahi_resolver_handler.h>
#endif
#include <netinet/in.h>
#include <sys/socket.h>

#include <cstddef>
#include <list>
#include <map>
#include <stdint.h>
#include <string>
#include <utility>

namespace fawkes {

class AvahiThread;
class NetworkNameResolver;
class Mutex;

#ifdef HAVE_AVAHI
class NetworkNameResolverThread : public Thread, public AvahiResolverHandler
#else
class NetworkNameResolverThread : public Thread
#endif
{
public:
	NetworkNameResolverThread(NetworkNameResolver *resolver, AvahiThread *avahi_thread = NULL);
	~NetworkNameResolverThread();

	void resolve_name(const std::string &name);
	void resolve_address(struct sockaddr *addr, socklen_t addrlen);

	bool
	     resolve_name_immediately(const std::string &name, struct sockaddr **addr, socklen_t *addr_len);
	bool resolve_address_immediately(struct sockaddr *addr, std::string &name, bool &namefound);

	virtual void resolved_name(char *name, struct sockaddr *addr, socklen_t addrlen);
	virtual void resolved_address(struct sockaddr *addr, socklen_t addrlen, char *name);
	virtual void name_resolution_failed(char *name);
	virtual void address_resolution_failed(struct sockaddr *addr, socklen_t addrlen);

	virtual void loop();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	NetworkNameResolver *resolver_;
#ifdef HAVE_AVAHI
	AvahiThread *avahi_thread_;
#endif

	Mutex *                          namesq_mutex_;
	unsigned int                     namesq_active_;
	typedef LockHashSet<std::string> NamesQMap;
	NamesQMap                        namesqs_[2];
	NamesQMap *                      namesq_;
	NamesQMap *                      namesq_proc_;

	Mutex *                              addrq_mutex_;
	unsigned int                         addrq_active_;
	typedef std::list<struct sockaddr *> AddrQList;
	AddrQList                            addrqs_[2];
	AddrQList *                          addrq_;
	AddrQList *                          addrq_proc_;
};

} // end namespace fawkes

#endif
