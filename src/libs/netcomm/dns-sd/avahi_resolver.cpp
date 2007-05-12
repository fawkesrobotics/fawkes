
/***************************************************************************
 *  avahi_resolver.cpp - Avahi name resolver
 *
 *  Created: Tue May 10 16:33:14 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>

#include <netcomm/dns-sd/avahi_resolver.h>

#include <avahi-common/error.h>
#include <netinet/in.h>
#include <utility>


/** @class AvahiResolverHandler <netcomm/dns-sd/avahi_resolver.h>
 * Avahi resolver handler interface.
 * This interface has to be implemented to make use of the threaded
 * Avahi lookup of names and addresses. After you have ordered a
 * lookup this handler is called with the result.
 * @author Tim Niemueller
 *
 * @fn void AvahiResolverHandler::resolved_name(char *name, struct sockaddr *addr, socklen_t addrlen) = 0
 * Name has been successfully resolved.
 * The ordered name lookup was successful for the given name resulting in
 * the given addr of addrlen bytes length.
 * Note that all of the parameters are given to the handler's ownership, that means
 * especially that the handler is responsible for freeing the associated memory
 * after it is done with the result using free() on name and addr.
 * @param name name that was resolved
 * @param addr resulting addr record, currently always of type struct sockaddr_in (only IPv4)
 * @param addrlen length of addr in bytes
 *
 * @fn void AvahiResolverHandler::resolved_address(struct sockaddr_in *addr, socklen_t addrlen, char *name) = 0
 * Address has been successfully resolved.
 * The ordered name lookup was successful for the given address resulting in
 * the given name.
 * Note that all of the parameters are given to the handler's ownership, that means
 * especially that the handler is responsible for freeing the associated memory
 * after it is done with the result using free() on name and addr.
 * @param name the resulting hostname
 * @param addr addr record, currently always of type struct sockaddr_in (only IPv4)
 * @param addrlen length of addr in bytes
 *
 * @fn void AvahiResolverHandler::name_resolution_failed(char *name) = 0
 * Name resolution failed.
 * The given hostname could not be resolved.
 * Note that the parameter name is given to the handler's ownership. This means
 * especially that the handler is responsible for freeing the memory with free()
 * after it is done with the variable.
 * @param name name whose lookup failed
 *
 * @fn void AvahiResolverHandler::address_resolution_failed(struct sockaddr_in *addr, socklen_t addrlen) = 0
 * Address resolution failed.
 * The given address could not be resolved.
 * Note that the parameter addr is given to the handler's ownership. This means
 * especially that the handler is responsible for freeing the memory with free()
 * after it is done with the variable.
 * @param addr address whose lookup failed
 * @param addrlen length of address
 */

/** Virtual empty destructor. */
AvahiResolverHandler::~AvahiResolverHandler()
{
}



/** @class AvahiResolver <netcomm/dns-sd/avahi_resolver.h>
 * Avahi resolver.
 * This utility provides methods for resolving names and addresses using
 * mDNS/DNS-SD via Avahi.
 * @author Tim Niemueller
 */

typedef std::pair<AvahiResolver *, AvahiResolverHandler *> AvahiResolverCallbackData;


/** Constructor. */
AvahiResolver::AvahiResolver()
{
  available = false;
  client = NULL;
  running_hostname_resolvers.clear();
  running_address_resolvers.clear();
}


/** Destructor.
 * This will free all still-running resolvers.
 */
AvahiResolver::~AvahiResolver()
{
  running_hostname_resolvers.lock();
  LockList<AvahiHostNameResolver *>::iterator i;
  for (i = running_hostname_resolvers.begin(); i != running_hostname_resolvers.end(); ++i) {
    avahi_host_name_resolver_free(*i);
  }
  running_hostname_resolvers.clear();
  running_hostname_resolvers.unlock();

  running_address_resolvers.lock();
  LockList<AvahiAddressResolver *>::iterator ai;
  for (ai = running_address_resolvers.begin(); ai != running_address_resolvers.end(); ++ai) {
    avahi_address_resolver_free(*ai);
  }
  running_address_resolvers.clear();
  running_address_resolvers.unlock();
}


/** Order name resolution.
 * This initiates resolution of a name. The method immediately returns and will not
 * wait for the result.
 * @param name name to resolve.
 * @param handler handler to call for the result
 */
void
AvahiResolver::resolve_name(const char *name, AvahiResolverHandler *handler)
{
  if ( ! available || ( client == NULL ) ) {
    throw Exception("AvahiResolver has not been initialized");
  }

  AvahiResolverCallbackData *data = new AvahiResolverCallbackData(this, handler);

  printf("Starting resolver for %s\n", name);
  AvahiHostNameResolver *resolver;
  if ( (resolver = avahi_host_name_resolver_new(client, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC,
						name, AVAHI_PROTO_INET,
						AVAHI_LOOKUP_USE_MULTICAST,
						AvahiResolver::host_name_resolver_callback,
						data) ) == NULL ) {
    throw Exception("Cannot create Avahi name resolver");
  } else {
    running_hostname_resolvers.push_back(resolver);
  }
}


/** Order address resolution.
 * This initiates resolution of an address. The method immediately returns and will not
 * wait for the result.
 * @param addr address to resolve, currently only struct sockaddr_in is supported (IPv4)
 * @param addrlen length of addr in bytes
 * @param handler handler to call for the result
 */
void
AvahiResolver::resolve_address(struct sockaddr *addr, socklen_t addrlen,
			       AvahiResolverHandler *handler)
{
  if ( ! available || ( client == NULL ) ) {
    throw Exception("AvahiResolver has not been initialized");
  }
  if ( addrlen != sizeof(struct sockaddr_in) ) {
    throw Exception("Only IPv4 is currently supported");
  }

  struct sockaddr_in *in_addr = (struct sockaddr_in *)addr;

  AvahiResolverCallbackData *data = new AvahiResolverCallbackData(this, handler);

  AvahiAddress a;
  a.proto = AVAHI_PROTO_INET;
  a.data.ipv4.address = in_addr->sin_addr.s_addr;

  AvahiAddressResolver *resolver;
  if ( (resolver = avahi_address_resolver_new(client, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC,
					      &a, AVAHI_LOOKUP_USE_MULTICAST,
					      AvahiResolver::address_resolver_callback,
					      data) ) == NULL ) {
    Exception e("Cannot create Avahi address resolver");
    e.append("Avahi error: %s", avahi_strerror(avahi_client_errno(client)));
    throw e;
  } else {
    running_address_resolvers.push_back(resolver);
  }
}


/** Set availability of service.
 * This is meant to be called only by AvahiThread depending on the availability
 * of the mDNS-SD service.
 * @param available true if available, alse otherwise
 */
void
AvahiResolver::set_available(bool available)
{
  this->available = available;
}


/** Remove hostname resolver.
 * Used internally by callback.
 * @param r resolver
 */
void
AvahiResolver::remove_hostname_resolver(AvahiHostNameResolver *r)
{
  running_hostname_resolvers.remove_locked(r);
}


/** Remove address resolver.
 * Used internally by callback.
 * @param r resolver
 */
void
AvahiResolver::remove_address_resolver(AvahiAddressResolver *r)
{
  running_address_resolvers.remove_locked(r);
}


/** Internal callback.
 * Callback for avahi.
 */
void
AvahiResolver::host_name_resolver_callback(AvahiHostNameResolver *r,
					   AvahiIfIndex interface,
					   AvahiProtocol protocol,
					   AvahiResolverEvent event,
					   const char *name,
					   const AvahiAddress *a,
					   AvahiLookupResultFlags flags,
					   void *userdata)
{
  AvahiResolverCallbackData *cd = static_cast<AvahiResolverCallbackData *>(userdata);

  cd->first->remove_hostname_resolver(r);
  avahi_host_name_resolver_free(r);

  switch (event) {
  case AVAHI_RESOLVER_FOUND:
    {
      struct sockaddr_in *res = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
      res->sin_family = avahi_proto_to_af(protocol);
      res->sin_addr.s_addr = a->data.ipv4.address;
      
      cd->second->resolved_name(strdup(name), (struct sockaddr *)res, sizeof(struct sockaddr_in));
    }
    break;
    
  case AVAHI_RESOLVER_FAILURE:
  default:
    cd->second->name_resolution_failed(strdup(name));
    break;
  }

  delete cd;
}


/** Internal callback.
 * Callback for avahi.
 */
void
AvahiResolver::address_resolver_callback(AvahiAddressResolver *r,
					 AvahiIfIndex interface,
					 AvahiProtocol protocol,
					 AvahiResolverEvent event,
					 const AvahiAddress *a,
					 const char *name,
					 AvahiLookupResultFlags flags,
					 void *userdata)
{
  AvahiResolverCallbackData *cd = static_cast<AvahiResolverCallbackData *>(userdata);

  cd->first->remove_address_resolver(r);
  avahi_address_resolver_free(r);

  struct sockaddr_in *res = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
  res->sin_family = avahi_proto_to_af(protocol);
  res->sin_addr.s_addr = a->data.ipv4.address;

   switch (event) {
  case AVAHI_RESOLVER_FOUND:
    cd->second->resolved_address((struct sockaddr_in *)res, sizeof(struct sockaddr_in),
				 strdup(name));
    break;
  case AVAHI_RESOLVER_FAILURE:
  default:
    cd->second->address_resolution_failed((struct sockaddr_in *)res,
					  sizeof(struct sockaddr_in));
    break;
  }

  delete cd;
}
