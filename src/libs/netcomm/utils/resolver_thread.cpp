
/***************************************************************************
 *  resolver_thread.cpp - Fawkes network name resolver thread
 *
 *  Created: Fri May 11 22:12:51 2007
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

#include <netcomm/utils/resolver_thread.h>
#include <netcomm/utils/resolver.h>
#ifdef HAVE_AVAHI
#include <netcomm/dns-sd/avahi_thread.h>
#endif
#include <core/exceptions/system.h>

#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <cstring>
#include <cstdlib>

namespace fawkes {

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
 * @ingroup NetComm
 * @author Tim Niemueller
 */


/** Constructor.
 * Available only if Avahi is available at compile time.
 * @param resolver network name resolver to call for results
 * @param avahi_thread Avahi thread, may be NULL in which case mDNS via
 * Avahi is not used.
 */
NetworkNameResolverThread::NetworkNameResolverThread(NetworkNameResolver *resolver,
						     AvahiThread *avahi_thread)
  : Thread("NetworkNameResolverThread", Thread::OPMODE_WAITFORWAKEUP)
{
  __resolver = resolver;
  __addrq_mutex   = new Mutex();
  __namesq_mutex  = new Mutex();

  __namesq_active = 0;
  __namesq        = &__namesqs[0];
  __namesq_proc   = &__namesqs[1];

  __addrq_active  = 0;
  __addrq         = &__addrqs[0];
  __addrq_proc    = &__addrqs[1];

#ifdef HAVE_AVAHI
  __avahi_thread = avahi_thread;
#endif
}

/** Destructor. */
NetworkNameResolverThread::~NetworkNameResolverThread()
{
  __namesq_mutex->lock();
  while ( ! __namesq->empty() ) {
    NamesQMap::iterator nqit = __namesq->begin();
    char *nqn = (*nqit);
    __namesq->erase(nqit);
    free(nqn);
  }
  while ( ! __namesq_proc->empty() ) {
    NamesQMap::iterator nqit = __namesq_proc->begin();
    char *nqn = (*nqit);
    __namesq->erase(nqit);
    free(nqn);
  }
  __namesq_mutex->unlock();
  __addrq_mutex->lock();
  while ( ! __addrq->empty() ) {
    AddrQMap::iterator nqit = __addrq->begin();
    free(nqit->second.first);
    __addrq->erase(nqit);
  }
  // The next operation cannot be locked, but we make the (valid) assumption
  // that the thread is not running when it is destructed, this situation is
  // an error anyway
  while ( ! __addrq_proc->empty() ) {
    AddrQMap::iterator nqit = __addrq_proc->begin();
    free(nqit->second.first);
    __addrq->erase(nqit);
  }
  __addrq_mutex->unlock();
  delete __addrq_mutex;
  delete __namesq_mutex;
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

#ifdef HAVE_AVAHI
  // resolve names in .local domain with Avahi if available
  char *n = (char *)name + strlen(name) - 6; // 6 == strlen(".local")
  const char *f = strstr(name, ".local");
  if ( __avahi_thread && f && (f == n) ) {
    __avahi_thread->resolve_name(name, this);          
  /*
  } else {
    printf("NOT ordering avahi_thread lookup\n");
    if ( ! avahi_thread ) 
      printf("No avahi resolver\n");
    if ( ! f ) {
      printf(".local not found\n");
    }
    if ( f != n ) {
      printf(".local at wrong location\n");
    }
  */
  }
#endif

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

#ifdef HAVE_AVAHI
  if ( __avahi_thread ) {
    __avahi_thread->resolve_address(addr, addr_len, this);
  }
#endif

  return found;
}


/** Enqueue name for resolution.
 * The name is enqueued and the resolver thread woken up. The result is reported
 * to the resolver given to the constructor.
 * @param name name to resolve
 */
void
NetworkNameResolverThread::resolve_name(const char *name)
{
  __namesq_mutex->lock();
  if ( __namesq->find((char *)name) == __namesq->end() ) {
    char *tmp = strdup(name);
    __namesq->insert(tmp);
    __namesq_mutex->unlock();
    wakeup();
  } else {
    __namesq_mutex->unlock();
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
  struct ::sockaddr_in *saddr = (struct ::sockaddr_in *)addr;
  __addrq_mutex->lock();
  if ( __addrq->find(saddr->sin_addr.s_addr) == __addrq->end() ) {
    struct sockaddr *taddr = (struct sockaddr *)malloc(addrlen);
    memcpy(taddr, addr, addrlen);
    (*__addrq)[saddr->sin_addr.s_addr] = std::make_pair(taddr, addrlen);
    __addrq_mutex->unlock();
    wakeup();
  } else {
    __addrq_mutex->unlock();
  }
}


/** Name has been successfully resolved.
 * The ordered name lookup was successful for the given name resulting in
 * the given addr of addrlen bytes length.
 * Note that all of the parameters are given to the handler's ownership, that means
 * especially that the handler is responsible for freeing the associated memory
 * after it is done with the result using free() on name and addr.
 * @param name name that was resolved
 * @param addr resulting addr record, currently always of type struct sockaddr_in (only IPv4)
 * @param addrlen length of addr in bytes
 */
void
NetworkNameResolverThread::resolved_name(char *name,
					 struct sockaddr *addr, socklen_t addrlen)
{
  __resolver->name_resolved(name, addr, addrlen);
}


/** Address has been successfully resolved.
 * The ordered name lookup was successful for the given address resulting in
 * the given name.
 * Note that all of the parameters are given to the handler's ownership, that means
 * especially that the handler is responsible for freeing the associated memory
 * after it is done with the result using free() on name and addr.
 * @param name the resulting hostname
 * @param addr addr record, currently always of type struct sockaddr_in (only IPv4)
 * @param addrlen length of addr in bytes
 */
void
NetworkNameResolverThread::resolved_address(struct sockaddr_in *addr, socklen_t addrlen,
					    char *name)
{
  __resolver->addr_resolved((struct sockaddr *)addr, addrlen, name, true);
}


/** Name resolution failed.
 * The given hostname could not be resolved.
 * Note that the parameter name is given to the handler's ownership. This means
 * especially that the handler is responsible for freeing the memory with free()
 * after it is done with the variable.
 * @param name name whose lookup failed
 */
void
NetworkNameResolverThread::name_resolution_failed(char *name)
{
  __resolver->name_resolution_failed(name);
}


/** Address resolution failed.
 * The given address could not be resolved.
 * Note that the parameter addr is given to the handler's ownership. This means
 * especially that the handler is responsible for freeing the memory with free()
 * after it is done with the variable.
 * @param addr address whose lookup failed
 * @param addrlen length of address
 */
void
NetworkNameResolverThread::address_resolution_failed(struct sockaddr_in *addr, socklen_t addrlen)
{
  __resolver->address_resolution_failed((struct sockaddr *)addr, addrlen);
}


/** Thread loop.
 * This will carry out all enqueued resolution operations.
 */
void
NetworkNameResolverThread::loop()
{
  __addrq_mutex->lock();
  __addrq_proc = __addrq;
  __addrq_active = 1 - __addrq_active;
  __addrq = &__addrqs[__addrq_active];
  __addrq_mutex->unlock();
  AddrQMap::iterator aqit;
  while ( ! __addrq_proc->empty() ) {
    aqit = __addrq_proc->begin();
    
    char *name;
    bool  namefound;

    if ( resolve_address_immediately(aqit->second.first, aqit->second.second, &name, &namefound) ) {
      __resolver->addr_resolved(aqit->second.first, aqit->second.second, name, namefound);
    } else {
      __resolver->address_resolution_failed(aqit->second.first, aqit->second.second);
    }
    __addrq_proc->erase(aqit);
  }

  __namesq_mutex->lock();
  __namesq_proc = __namesq;
  __namesq_active = 1 - __namesq_active;
  __namesq = &__namesqs[__namesq_active];
  __namesq_mutex->unlock();
  NamesQMap::iterator nqit;
  while ( ! __namesq_proc->empty() ) {
    nqit = __namesq_proc->begin();
    char *nqn = (*nqit);

    struct sockaddr *addr;
    socklen_t addrlen;

    // we strdup here because otherwise we could not ensure
    // that the erase operation below can still suceed!
    // And even if we make it mandatory that the name_resolved will not
    // free the memory we would have the problem that it would be
    // unknown when the resolver may free the variable
    if ( resolve_name_immediately(nqn, &addr, &addrlen) ) {
      __resolver->name_resolved(strdup(nqn), addr, addrlen);
    } else {
      __resolver->name_resolution_failed(strdup(nqn));
    }
    __namesq_proc->erase(nqit);
    free(nqn);
  }
}

} // end namespace fawkes
