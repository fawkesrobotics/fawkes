
/***************************************************************************
 *  avahi_resolver.h - Avahi name resolver
 *
 *  Created: Tue Nov 14 14:32:56 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_DNSSD_AVAHI_RESOLVER_H_
#define __NETCOMM_DNSSD_AVAHI_RESOLVER_H_

#include <core/utils/lock_list.h>

#include <sys/socket.h>
#include <avahi-client/client.h>
#include <avahi-client/lookup.h>

class AvahiResolverHandler
{
 public:
  virtual ~AvahiResolverHandler();
  virtual void resolved_name(char *name,
			     struct sockaddr *addr, socklen_t addrlen)   = 0;
  virtual void resolved_address(struct sockaddr_in *addr, socklen_t addrlen,
				char *name)                              = 0;

  virtual void name_resolution_failed(char *name)                        = 0;
  virtual void address_resolution_failed(struct sockaddr_in *addr,
					 socklen_t addrlen)              = 0;
};

class AvahiResolver
{
  friend class AvahiThread;

 public:
  AvahiResolver();
  ~AvahiResolver();

  void resolve_name(const char *name, AvahiResolverHandler *handler);
  void resolve_address(struct sockaddr *addr, socklen_t addrlen,
		       AvahiResolverHandler *handler);

  /*
  bool resolve_name(const char *name, struct sockaddr **addr, socklen_t *addrlen);
  bool resolve_address(struct sockaddr *addr, socklen_t addrlen, char **name);
  */

 private:
  void set_available(bool available);

  void remove_hostname_resolver(AvahiHostNameResolver *r);
  void remove_address_resolver(AvahiAddressResolver *r);

  static void host_name_resolver_callback(AvahiHostNameResolver *r,
					  AvahiIfIndex interface,
					  AvahiProtocol protocol,
					   AvahiResolverEvent event,
					  const char *name,
					  const AvahiAddress *a,
					  AvahiLookupResultFlags flags,
					  void *userdata);
  
  static void address_resolver_callback(AvahiAddressResolver *r,
					AvahiIfIndex interface,
					AvahiProtocol protocol,
					AvahiResolverEvent event,
					const AvahiAddress *a,
					const char *name,
					AvahiLookupResultFlags flags,
					void *userdata);

  AvahiClient    *client;

  bool available;

  LockList<AvahiHostNameResolver *> running_hostname_resolvers;
  LockList<AvahiAddressResolver *>  running_address_resolvers;
};

#endif
