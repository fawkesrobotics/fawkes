
/***************************************************************************
 *  resolver_thread.cpp - Fawkes network name resolver thread
 *
 *  Created: Fri May 11 22:12:51 2007
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

#include <netcomm/utils/resolver_thread.h>
#include <netcomm/utils/resolver.h>
#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/dns-sd/avahi_resolver.h>
#include <core/exceptions/system.h>

#include <sys/types.h>
#include <netdb.h>
#include <cstring>
#include <cstdlib>


/** @class NetworkNameResolverThread <netcomm/utils/resolver_thread.h>
 * Worker thread for NetworkNameResolver.
 * This thread does the work for the NetworkNameResolver. It runs concurrently
 * to the rest of the software and executes name and address lookups in a
 * non-blocking fashion.
 *
 * This class should not be used directly, but NetworkNameResolver should
 * be used instead.
 *
 * @see NetworkNameResolver
 * @author Tim Niemueller
 */


/** Constructor.
 * @param resolver network name resolver to call for results
 * @param avahi_thread Avahi thread, may be NULL in which case mDNS via
 * Avahi is not used.
 */
NetworkNameResolverThread::NetworkNameResolverThread(NetworkNameResolver *resolver,
						     AvahiThread *avahi_thread)
  : Thread("NetworkNameResolverThread", Thread::OPMODE_WAITFORWAKEUP)
{
  this->resolver = resolver;

  if ( avahi_thread != NULL ) {
    avahi_resolver = avahi_thread->resolver();
  } else {
    avahi_resolver = NULL;
  }
  namesq.clear();
}


/** Destructor. */
NetworkNameResolverThread::~NetworkNameResolverThread()
{
  namesq.lock();
  while ( ! namesq.empty() ) {
    nqit = namesq.begin();
    char *nqn = (*nqit);
    namesq.erase(nqit);
    free(nqn);
  }
  namesq.unlock();
}


/** Immediately resolve a name.
 * This tries to lookup a name with the getaddrinfo() and if the name ends with
 * .local (the host is in the .local domain) and an Avahi thread has been supplied
 * Avahi is used to lookup the hostname as well, but this does not happen immediately
 * because this can take some time.
 * @param name host name to lookup
 * @param addr upon return and success the address result will be stored here in a
 * newly allocated buffer which you have to free after use using free().
 * @param addr_len upon return and success contains the length of addr in bytes
 * @return true if the name has been successfully resolved in which case addr and
 * addr_len carry the result, false otherwise
 */
bool
NetworkNameResolverThread::resolve_name_immediately(const char *name,
						    struct sockaddr **addr, socklen_t *addr_len)
{
  bool found = false;

  // First try a regular lookup
  struct addrinfo *ai;
  if ( getaddrinfo(name, NULL, NULL, &ai) == 0 ) {
    // return the first result
    struct sockaddr *tmp = (struct sockaddr *)malloc(ai->ai_addrlen);
    memcpy(tmp, ai->ai_addr, ai->ai_addrlen);
    *addr = tmp;
    *addr_len = ai->ai_addrlen;
    freeaddrinfo(ai);
    found = true;
  }

  // resolve names in .local domain with Avahi if available
  char *n = (char *)name + strlen(name) - 6; // 6 == strlen(".local")
  char *f = strstr(name, ".local");
  if ( avahi_resolver && f && (f == n) ) {
    avahi_resolver->resolve_name(name, this);          
  /*
  } else {
    printf("NOT ordering avahi_resolver lookup\n");
    if ( ! avahi_resolver ) 
      printf("No avahi resolver\n");
    if ( ! f ) {
      printf(".local not found\n");
    }
    if ( f != n ) {
      printf(".local at wrong location\n");
    }
  */
  }

  return found;
}


/** Immediately resolve address.
 * This tries to lookup the address with the getnameinfo(). If that fails a textual
 * representation of the address is created. Additionally if an Avahi thread has
 * @param addr pointer to a struct of type struct sockaddr_in with the address to
 * lookup
 * @param addr_len length of addr in bytes
 * @param name contains a newly allocated buffer upon successful return that you have
 * to free after use using free().
 * @param namefound true, if the name could be resolved, false if it was just transformed
 * to a textual representation
 * @return true if the address has been successfully resolved in which case name
 * carries the result, false otherwise
 */
bool
NetworkNameResolverThread::resolve_address_immediately(struct sockaddr *addr, socklen_t addr_len,
						       char **name, bool *namefound)
{
  bool found = false;
  char hbuf[NI_MAXHOST];
  if ( getnameinfo(addr, addr_len, hbuf, sizeof(hbuf), NULL, 0, NI_NAMEREQD) == 0 ) {
    char *tmp = (char *)malloc(strlen(hbuf) + 1);
    if ( ! tmp ) {
      throw OutOfMemoryException();
    }
    strcpy(tmp, hbuf);
    *name = tmp;
    *namefound = true;
    found = true;
  } else if ( getnameinfo(addr, addr_len, hbuf, sizeof(hbuf), NULL, 0, 0) == 0 ) {
    char *tmp = (char *)malloc(strlen(hbuf) + 1);
    if ( ! tmp ) {
      throw OutOfMemoryException();
    }
    strcpy(tmp, hbuf);
    *name = tmp;
    *namefound = false;
    found = true;
  }

  if ( avahi_resolver ) {
    avahi_resolver->resolve_address(addr, addr_len, this);
  }

  return found;
}


/** Enqueue name for resolution.
 * The name is enqueued and the resolver thread woken up. The result is reported
 * to the resolver given to the constructor.
 * @param name name to resolve
 */
void
NetworkNameResolverThread::resolve_name(char *name)
{
  namesq.lock();
  if ( namesq.find(name) == namesq.end() ) {
    namesq.insert(strdup(name));
    namesq.unlock();
    wakeup();
  } else {
    namesq.unlock();
  }
}


/** Enqueue address for resolution.
 * The address is enqueued and the resolver thread woken up. The result is reported
 * to the resolver given to the constructor.
 * @param addr address to resolve, must be a struct sockaddr_in
 * @param addrlen length of addr
 */
void
NetworkNameResolverThread::resolve_address(struct sockaddr *addr, socklen_t addrlen)
{
  struct sockaddr_in *saddr = (struct sockaddr_in *)addr;
  addrq.lock();
  if ( addrq.find(saddr->sin_addr.s_addr) == addrq.end() ) {
    struct sockaddr *taddr = (struct sockaddr *)malloc(addrlen);
    memcpy(taddr, addr, addrlen);
    addrq[saddr->sin_addr.s_addr] = std::pair<struct sockaddr *, socklen_t>(taddr, addrlen);
    addrq.unlock();
    wakeup();
  } else {
    addrq.unlock();
  }
}


void
NetworkNameResolverThread::resolved_name(char *name,
					 struct sockaddr *addr, socklen_t addrlen)
{
  resolver->name_resolved(name, addr, addrlen);
}


void
NetworkNameResolverThread::resolved_address(struct sockaddr_in *addr, socklen_t addrlen,
					    char *name)
{
  printf("DEB Address resolved to %s by Avahi\n", name);
  resolver->addr_resolved((struct sockaddr *)addr, addrlen, name, true);
}

void
NetworkNameResolverThread::name_resolution_failed(char *name)
{
  resolver->name_resolution_failed(name);
}


void
NetworkNameResolverThread::address_resolution_failed(struct sockaddr_in *addr, socklen_t addrlen)
{
  printf("DEB Avahi address resolution failed\n");
  resolver->address_resolution_failed((struct sockaddr *)addr, addrlen);
}


/** Thread loop.
 * This will carry out all enqueued resolution operations.
 */
void
NetworkNameResolverThread::loop()
{
  addrq.lock();
  while ( ! addrq.empty() ) {
    aqit = addrq.begin();
    
    char *name;
    bool  namefound;

    if ( resolve_address_immediately((*aqit).second.first, (*aqit).second.second, &name, &namefound) ) {
      resolver->addr_resolved((*aqit).second.first, (*aqit).second.second, name, namefound);
    } else {
      resolver->address_resolution_failed((*aqit).second.first, (*aqit).second.second);
    }
    addrq.erase(aqit);
  }
  addrq.unlock();

  namesq.lock();
  while ( ! namesq.empty() ) {
    nqit = namesq.begin();
    char *nqn = (*nqit);
    
    struct sockaddr *addr;
    socklen_t addrlen;

    // we strdup here because otherwise we could not ensure
    // that the erase operation below can still suceed!
    // And even if we make it mandatory that the name_resolved will not
    // free the memory we would have the problem that it would be
    // unknown when the resolver may free the variable
    if ( resolve_name_immediately(nqn, &addr, &addrlen) ) {
      resolver->name_resolved(strdup(nqn), addr, addrlen);
    } else {
      resolver->name_resolution_failed(strdup(nqn));
    }
    namesq.erase(nqit);
    free(nqn);
  }
  namesq.unlock();
}
