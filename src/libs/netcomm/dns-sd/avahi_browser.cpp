
/***************************************************************************
 *  avahi_browser.cpp - browse services via Avahi
 *
 *  Created: Wed Nov 08 13:19:41 2006
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

#include <netcomm/dns-sd/avahi_browser.h>

#include <core/threading/mutex.h>
#include <core/exceptions/software.h>

#include <avahi-common/cdecl.h>
#include <avahi-common/defs.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>


/** @class AvahiBrowser netcomm/dns-sd/avahi_browser.h
 * Avahi service browser.
 * Encapsulates the avahi service browser. You register a result handler and
 * then initiate searches. The result handler is then informed about
 * the results.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */


/** Constructor. */
AvahiBrowser::AvahiBrowser()
{
  mutex = new Mutex();
  client = NULL;
}

/** Destructor. */
AvahiBrowser::~AvahiBrowser()
{
  delete mutex;
}


/** Add a result handler.
 * A handler is added for the given service type. A search is initiated
 * for the given service and the given handler is called for added or
 * removed services or if an error occurs.
 * @param service_type string of the service type
 * @param h The ServiceBrowseHandler
 */
void
AvahiBrowser::add_handler(const char *service_type, ServiceBrowseHandler *h)
{
  //mutex->lock();
  handlers[service_type].push_back(h);
  create_browser(service_type);
  //mutex->unlock();
}


/** Create browser for a given service.
 * @param service_type service type
 */
void
AvahiBrowser::create_browser(const char *service_type)
{
  if ( browsers.find(service_type) == browsers.end() ) {
    if ( client ) {
      AvahiServiceBrowser *b = avahi_service_browser_new(client, AVAHI_IF_UNSPEC,
							 AVAHI_PROTO_UNSPEC,
							 service_type, NULL, (AvahiLookupFlags)0,
							 AvahiBrowser::browse_callback, this);

      if ( ! b ) {
	handlers[service_type].pop_back();
	throw NullPointerException("Could not instantiate AvahiServiceBrowser");
      }
      browsers[service_type] = b;
    }
  }
}


/** Create browsers.
 * Creates browser for all services.
 */
void
AvahiBrowser::create_browsers()
{
  std::map< std::string, std::list<ServiceBrowseHandler *> >::iterator i;
  for (i = handlers.begin(); i != handlers.end(); ++i) {
    create_browser( (*i).first.c_str() );
  }
}


/** Erase all browsers. */
void
AvahiBrowser::erase_browsers()
{
  std::map< std::string, AvahiServiceBrowser * >::iterator i;
  for (i = browsers.begin(); i != browsers.end(); ++i) {
    avahi_service_browser_free((*i).second);
  }
  browsers.clear();
}


/** Remove a handler.
 * The handler is removed and no further events will be emitted to the
 * handler.
 * @param service_type service type to de-register the handler for
 * @param h the handler
 */
void
AvahiBrowser::remove_handler(const char *service_type, ServiceBrowseHandler *h)
{
  //mutex->lock();
  if ( handlers.find(service_type) != handlers.end() ) {
    handlers[service_type].remove(h);
    if ( handlers[service_type].size() == 0 ) {
      if ( browsers.find(service_type) != browsers.end() ) {
	avahi_service_browser_free(browsers[service_type]);
	browsers.erase(service_type);
      }
      handlers.erase(service_type);
    }
  }
  //mutex->unlock();
}


/** Call handler for a removed service.
 * @param name name
 * @param type type
 * @param domain domain
 */
void
AvahiBrowser::call_handler_service_removed( const char *name,
					    const char *type,
					    const char *domain)
{
  //mutex->lock();
  if ( handlers.find(type) != handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = handlers[type].begin(); i != handlers[type].end(); ++i) {
      (*i)->service_removed(name, type, domain);
    }
  }
  //mutex->unlock();
}


/** Call handler for an added service.
 * @param name name
 * @param type type
 * @param domain domain
 * @param host_name host name
 * @param address address of host
 * @param port port of service
 * @þaram txt list of TXT records
 * @param flags flags
 */
void
AvahiBrowser::call_handler_service_added( const char *name,
					  const char *type,
					  const char *domain,
					  const char *host_name,
					  const AvahiAddress *address,
					  uint16_t port,
					  std::list<std::string> &txt,
					  AvahiLookupResultFlags flags)
{
  //mutex->lock();
  struct sockaddr_in *s = NULL;
  socklen_t slen;
  if ( address->proto == AVAHI_PROTO_INET ) {
    slen = sizeof(struct sockaddr_in);
    s = (struct sockaddr_in *)malloc(slen);
    s->sin_addr.s_addr = address->data.ipv4.address;
  } else {
    // ignore
    return;
  }
  if ( handlers.find(type) != handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = handlers[type].begin(); i != handlers[type].end(); ++i) {
      (*i)->service_added(name, type, domain, host_name,
			  (struct sockaddr *)s, slen, port, txt, (int)flags);
    }
  }
  free(s);
  //mutex->unlock();
}


/** Call handler for failure.
 * @param name name
 * @param type type
 * @param domain domain
 */
void
AvahiBrowser::call_handler_failed( const char *name,
				   const char *type,
				   const char *domain)
{
  //mutex->lock();
  if ( handlers.find(type) != handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = handlers[type].begin(); i != handlers[type].end(); ++i) {
      (*i)->browse_failed(name, type, domain);
    }
  }
  //mutex->unlock();
}


/** Call handler "all for now".
 * @param type type
 */
void
AvahiBrowser::call_handler_all_for_now(const char *type)
{
  //mutex->lock();
  if ( handlers.find(type) != handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = handlers[type].begin(); i != handlers[type].end(); ++i) {
      (*i)->all_for_now();
    }
  }
  //mutex->unlock();
}


/** Call handler "cache exhausted".
 * @param type type
 */
void
AvahiBrowser::call_handler_cache_exhausted(const char *type)
{
  //mutex->lock();
  if ( handlers.find(type) != handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = handlers[type].begin(); i != handlers[type].end(); ++i) {
      (*i)->cache_exhausted();
    }
  }
  //mutex->unlock();
}


/** Callback for Avahi.
 * Callback called by Avahi.
 * @param b service browser
 * @param interface interface index
 * @param protocol protocol
 * @param event event
 * @param name name
 * @param type type
 * @param domain domain
 * @param flags flags
 * @param instance pointer to the AvahiBrowser instance that initiated
 * the search
 */
void
AvahiBrowser::browse_callback( AvahiServiceBrowser *b,
			       AvahiIfIndex interface,
			       AvahiProtocol protocol,
			       AvahiBrowserEvent event,
			       const char *name,
			       const char *type,
			       const char *domain,
			       AvahiLookupResultFlags flags,
			       void *instance)
{
  AvahiBrowser *ab = static_cast<AvahiBrowser *>(instance);

  switch (event) {
  case AVAHI_BROWSER_FAILURE:
    //printf("(Browser) %s\n", avahi_strerror(avahi_client_errno(avahi_service_browser_get_client(b))));
    return;

  case AVAHI_BROWSER_NEW:
    //printf("(Browser) NEW: service '%s' of type '%s' in domain '%s'\n", name, type, domain);
    // We ignore the returned resolver object. In the callback
    // function we free it. If the server is terminated before
    // the callback function is called the server will free
    // the resolver for us.
    if (!(avahi_service_resolver_new(ab->client, interface, protocol,
				     name, type, domain, protocol, (AvahiLookupFlags)0,
				     AvahiBrowser::resolve_callback, instance))) {
      throw NullPointerException("Could not instantiate resolver");
    }
    break;

  case AVAHI_BROWSER_REMOVE:
    // handler
    //printf("(Browser) REMOVE: service '%s' of type '%s' in domain '%s'\n", name, type, domain);
    ab->call_handler_service_removed(name, type, domain);
    break;

  case AVAHI_BROWSER_ALL_FOR_NOW:
    // handler
    //printf("(Browser) ALL_FOR_NOW: service '%s' of type '%s' in domain '%s'\n", name, type, domain);
    ab->call_handler_all_for_now(type);
    break;

  case AVAHI_BROWSER_CACHE_EXHAUSTED:
    // handler
    //printf("(Browser) CACHE_EXHAUSTED: service '%s' of type '%s' in domain '%s'\n", name, type, domain);
    ab->call_handler_cache_exhausted(type);
    break;

  }
}


/** Callback for Avahi.
 * Callback called by Avahi.
 * @param r service resolver
 * @param interface interface index
 * @param protocol protocol
 * @param event event
 * @param name name
 * @param type type
 * @param domain domain
 * @param host_name host name
 * @param address address
 * @param port port
 * @param txt TXT records
 * @param flags flags
 * @param instance pointer to the AvahiBrowser instance that initiated
 * the search
 */
void
AvahiBrowser::resolve_callback( AvahiServiceResolver *r,
				AVAHI_GCC_UNUSED AvahiIfIndex interface,
				AVAHI_GCC_UNUSED AvahiProtocol protocol,
				AvahiResolverEvent event,
				const char *name,
				const char *type,
				const char *domain,
				const char *host_name,
				const AvahiAddress *address,
				uint16_t port,
				AvahiStringList *txt,
				AvahiLookupResultFlags flags,
				void *instance)
{
  AvahiBrowser *ab = static_cast<AvahiBrowser *>(instance);

  switch (event) {
  case AVAHI_RESOLVER_FAILURE:
    // handler failure
    ab->call_handler_failed(name, type, domain);
    break;

  case AVAHI_RESOLVER_FOUND:
    // handler add
    {
      std::list< std::string > txts;
      AvahiStringList *l = txt;

      txts.clear();
      while ( l ) {
	txts.push_back((char *)avahi_string_list_get_text(l));
	l = avahi_string_list_get_next( l );
      }

      ab->call_handler_service_added(name, type, domain, host_name, address, port, txts, flags);
    }
    break;
  }

  avahi_service_resolver_free(r);
}
