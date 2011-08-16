
/***************************************************************************
 *  avahi_thread.cpp - Avahi thread
 *
 *  Created: Wed Nov 08 11:19:25 2006
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/dns-sd/avahi_resolver_handler.h>

#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#include <core/exceptions/software.h>
#include <utils/misc/string_conversions.h>

#include <avahi-client/lookup.h>
#include <avahi-client/publish.h>
#include <avahi-common/alternative.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>
#include <avahi-common/timeval.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <cstdlib>
#include <cstddef>
#include <cstring>

namespace fawkes {

/** @class AvahiThread netcomm/dns-sd/avahi_thread.h
 * Avahi main thread.
 * This thread handles all tasks related to avahi. This is the single
 * interaction point with the Avahi adapter.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Constructor. */
AvahiThread::AvahiThread()
  : Thread("AvahiThread")
{
  simple_poll = NULL;
  client = NULL;

  need_recover = false;
  do_reset_groups = false;

  init_wc = new WaitCondition();

  set_prepfin_conc_loop(true);
}


/** Destructor. */
AvahiThread::~AvahiThread()
{
  delete init_wc;

  remove_pending_services();
  remove_pending_browsers();

  erase_groups();
  erase_browsers();

  if ( client )
    avahi_client_free( client );

  if ( simple_poll )
    avahi_simple_poll_free( simple_poll );

}


/** Avahi thread loop.
 * The avahi thread calls the simple poll iterate to poll with an infinite
 * timeout. This way the loop blocks until an event occurs.
 */
void
AvahiThread::loop()
{
  if ( need_recover ) {
    if ( client ) {
      avahi_client_free( client );
      client = NULL;
    }

    if ( simple_poll ) {
      avahi_simple_poll_free( simple_poll );
      simple_poll = NULL;
    }
  }

  if ( ! simple_poll ) {
    // Init
    int error;

    if ( (simple_poll = avahi_simple_poll_new()) ) {

      client = avahi_client_new( avahi_simple_poll_get(simple_poll), AVAHI_CLIENT_NO_FAIL,
				 AvahiThread::client_callback, this, &error );

      if ( ! client ) {
	avahi_simple_poll_free( simple_poll );
      }
    }
  }

  if ( client ) {
    if ( do_reset_groups ) {
      reset_groups();
      recreate_services();
    }
    if ( need_recover ) {
      erase_groups();
      erase_browsers();
      recreate_services();
      recreate_browsers();
    }
    if ( client_state == AVAHI_CLIENT_S_RUNNING ) {
      remove_pending_services();
      remove_pending_browsers();
      create_pending_services();
      create_pending_browsers();
      start_hostname_resolvers();
      start_address_resolvers();
    }

    need_recover = false;

    avahi_simple_poll_iterate( simple_poll, -1);
  }
}


/** Recover froma broken Avahi connection.
 * This will erase all service browsers and announced service groups
 * and will try to reconnect in the next loop.
 */
void
AvahiThread::recover()
{
  need_recover = true;
  wake_poller();
}

void
AvahiThread::wake_poller()
{
  if ( simple_poll ) {
    avahi_simple_poll_wakeup( simple_poll );
  }
}


/** Called whenever the client or server state changes.
 * @param c Avahi client
 * @param state new state
 * @param instance Instance of AvahiThread that triggered the event.
 */
void
AvahiThread::client_callback(AvahiClient *c, AvahiClientState state, void *instance)
{
  AvahiThread *at = static_cast<AvahiThread *>(instance);
  at->client_state = state;

  switch (state) {
  case AVAHI_CLIENT_S_RUNNING:        
    /* The server has startup successfully and registered its host
     * name on the network, so it's time to create our services */
    //printf("(Client): RUNNING\n");
    //at->create_browsers();
    //at->set_available( true );
    at->init_done();
    break;

  case AVAHI_CLIENT_S_COLLISION:
    //printf("(Client): COLLISION\n");
    /* Let's drop our registered services. When the server is back
     * in AVAHI_SERVER_RUNNING state we will register them
     * again with the new host name. */
    at->do_reset_groups = true;
    break;
            
  case AVAHI_CLIENT_FAILURE:          
    // Doh!
    //printf("(Client): FAILURE\n");
    at->recover();
    break;

  case AVAHI_CLIENT_CONNECTING:
    //printf("(Client): CONNECTING\n");
    break;

  case AVAHI_CLIENT_S_REGISTERING:
    // Ignored
    //printf("(Client): REGISTERING\n");
    break;
  }
}

/* **********************************************************************************
 * Avahi Service Publisher methods
 * **********************************************************************************/


/** Publish service.
 * @param service service to publish.
 */
void
AvahiThread::publish_service(NetworkService *service)
{
  if ( __services.find(service) == __services.end() ) {
    __pending_services.push_locked(service);
  } else {
    throw Exception("Service already registered");
  }

  wake_poller();
}


void
AvahiThread::unpublish_service(NetworkService *service)
{
  if ( __services.find(service) != __services.end() ) {
    __pending_remove_services.push_locked(service);
  } else {
    throw Exception("Service not registered");
  }

  wake_poller();
}


/** Create services. */
AvahiEntryGroup *
AvahiThread::create_service(const NetworkService &service, AvahiEntryGroup *exgroup)
{
  // the following errors are non-fatal, they can happen since Avahi is started
  // asynchronously, just ignore them by bailing out
  if ( ! client )  return NULL;

  AvahiEntryGroup *group;
  if ( exgroup ) {
    group = exgroup;
  } else {
    if ( ! (group = avahi_entry_group_new(client,
					  AvahiThread::entry_group_callback,
					  this))) {
      throw NullPointerException("Cannot create service group");
    }
  }

  AvahiStringList *al = NULL;
  const std::list<std::string> &l = service.txt();
  for (std::list<std::string>::const_iterator j = l.begin(); j != l.end(); ++j) {
    al = avahi_string_list_add(al, j->c_str());
  }

  // only IPv4 for now
  int rv = AVAHI_ERR_COLLISION;
  for (int i = 1; (i <= 100) && (rv == AVAHI_ERR_COLLISION); ++i) {
    std::string name = service.name();
    if (i > 1) {
      name += " ";
      name += StringConversions::to_string(i);
    }
    
    rv = avahi_entry_group_add_service_strlst(group, AVAHI_IF_UNSPEC,
					      AVAHI_PROTO_INET,
					      AVAHI_PUBLISH_USE_MULTICAST,
					      name.c_str(), service.type(),
					      service.domain(),
					      service.host(),
					      service.port(), al);

    if ((i > 1) && (rv >= 0)) {
      service.set_modified_name(name.c_str());
    }
  }

  avahi_string_list_free(al);

  if (rv < 0) {
    throw Exception("Adding Avahi/mDNS-SD service failed: %s", avahi_strerror(rv));
  }

  /*
  if (service.modified_name() != 0) {
    LibLogger::log_warn("FawkesNetworkManager", "Network service name collision, "
			"modified to '%s' (from '%s')", service.modified_name(),
			service.name());
  }
  */

  /* Tell the server to register the service */
  if (avahi_entry_group_commit(group) < 0) {
    throw Exception("Registering Avahi services failed");
  }

  return group;
}

void
AvahiThread::recreate_services()
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    (*__sit).second = create_service(__sit->first, __sit->second);
  }
}


void
AvahiThread::create_pending_services()
{
  __pending_services.lock();
  while ( ! __pending_services.empty()) {
    NetworkService &s = __pending_services.front();
    __services[s] = create_service(s, NULL);
    __pending_services.pop();
  }
  __pending_services.unlock();
}


void
AvahiThread::remove_pending_services()
{
  Thread::CancelState old_state;
  set_cancel_state(CANCEL_DISABLED, &old_state);
  __pending_remove_services.lock();
  while ( ! __pending_remove_services.empty()) {
    NetworkService &s = __pending_remove_services.front();
    if ( __services.find(s) != __services.end() ) {
      group_erase(__services[s]);
      __services.erase_locked(s);
    }
    __pending_remove_services.pop();
  }
  __pending_remove_services.unlock();
  set_cancel_state(old_state);
}


/** Drop our registered services.
 * When the server is back in AVAHI_SERVER_RUNNING state we will register them
 * again with the new host name (triggered by AvahiThread).
 */
void
AvahiThread::group_reset(AvahiEntryGroup *g)
{
  if ( g ) {
    avahi_entry_group_reset(g);
  }
}


/** Erase service group. */
void
AvahiThread::group_erase(AvahiEntryGroup *g)
{
  if ( g ) {
    avahi_entry_group_reset( g );
    avahi_entry_group_free( g );
  }
}


void
AvahiThread::erase_groups()
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    if (__sit->second)  group_erase(__sit->second);
    __sit->second = NULL;
  }
}


void
AvahiThread::reset_groups()
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    group_reset((*__sit).second);
  }
}


/** Called if there was a name collision. */
void
AvahiThread::name_collision(AvahiEntryGroup *g)
{
  for (__sit = __services.begin(); __sit != __services.end(); ++__sit) {
    if ( (*__sit).second == g ) {
      NetworkService alternate_service((*__sit).first);

      /* A service name collision happened. Let's pick a new name */
      char *n = avahi_alternative_service_name((*__sit).first.name());
      alternate_service.set_name(n);
      avahi_free(n);

      __pending_remove_services.push_locked((*__sit).first);
      __pending_services.push_locked(alternate_service);
    }
  }
}


/** Callback for Avahi.
 * @param g entry group
 * @param state new state
 * @param instance instance of AvahiThread that triggered the event.
 */
void
AvahiThread::entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state,
				  void *instance)
{
  AvahiThread *at = static_cast<AvahiThread *>(instance);

  switch (state) {
  case AVAHI_ENTRY_GROUP_ESTABLISHED :
    /* The entry group has been established successfully */
    //fprintf(stderr, "Service '%s' successfully established.\n", name);
    break;

  case AVAHI_ENTRY_GROUP_COLLISION : {
    at->name_collision(g);
    break;
  }
  
  case AVAHI_ENTRY_GROUP_FAILURE :
    /* Some kind of failure happened while we were registering our services */
    at->recover();
    break;

  case AVAHI_ENTRY_GROUP_UNCOMMITED:
  case AVAHI_ENTRY_GROUP_REGISTERING:
    break;
  }
}


/* **********************************************************************************
 * Avahi Browser Publisher methods
 * **********************************************************************************/


/** Add a result handler.
 * A handler is added for the given service type. A search is initiated
 * for the given service and the given handler is called for added or
 * removed services or if an error occurs.
 * @param service_type string of the service type
 * @param h The ServiceBrowseHandler
 */
void
AvahiThread::watch_service(const char *service_type, ServiceBrowseHandler *h)
{
  __handlers[service_type].push_back(h);
  __pending_browsers.push_locked(service_type);

  wake_poller();
}


/** Remove a handler.
 * The handler is removed and no further events will be emitted to the
 * handler.
 * @param service_type service type to de-register the handler for
 * @param h the handler
 */
void
AvahiThread::unwatch_service(const char *service_type, ServiceBrowseHandler *h)
{
  if ( __handlers.find(service_type) != __handlers.end() ) {
    __handlers[service_type].remove(h);
    if ( __handlers[service_type].size() == 0 ) {
      if ( __browsers.find(service_type) != __browsers.end() ) {
	__pending_browser_removes.push_locked(service_type);
	//avahi_service_browser_free(__browsers[service_type]);
	//__browsers.erase(service_type);
      }
      __handlers.erase(service_type);
    }
  }

  wake_poller();
}


/** Create browser for a given service.
 * @param service_type service type
 */
void
AvahiThread::create_browser(const char *service_type)
{
  if ( __browsers.find(service_type) == __browsers.end() ) {
    if ( client ) {
      AvahiServiceBrowser *b = avahi_service_browser_new(client, AVAHI_IF_UNSPEC,
							 AVAHI_PROTO_UNSPEC,
							 service_type, NULL, (AvahiLookupFlags)0,
							 AvahiThread::browse_callback, this);

      if ( ! b ) {
	__handlers[service_type].pop_back();
	throw NullPointerException("Could not instantiate AvahiServiceBrowser");
      }
      __browsers[service_type] = b;
    }
  }
}


/** Create browsers.
 * Creates browser for all services.
 */
void
AvahiThread::recreate_browsers()
{
  LockMap< std::string, std::list<ServiceBrowseHandler *> >::iterator i;
  for (i = __handlers.begin(); i != __handlers.end(); ++i) {
    create_browser( (*i).first.c_str() );
  }
}


void
AvahiThread::create_pending_browsers()
{
  __pending_browsers.lock();
  while ( ! __pending_browsers.empty() ) {
    //printf("Creating browser for %s\n", __pending_browsers.front().c_str());
    create_browser(__pending_browsers.front().c_str());
    __pending_browsers.pop();
  }
  __pending_browsers.unlock();
}


void
AvahiThread::remove_pending_browsers()
{
  Thread::CancelState old_state;
  set_cancel_state(CANCEL_DISABLED, &old_state);
  __pending_browser_removes.lock();
  while ( ! __pending_browser_removes.empty()) {
    std::string &s = __pending_browser_removes.front();
    avahi_service_browser_free(__browsers[s]);
    __browsers.erase_locked(s);
    __pending_browser_removes.pop();
  }
  __pending_browser_removes.unlock();
  set_cancel_state(old_state);
}


/** Erase all browsers. */
void
AvahiThread::erase_browsers()
{
  std::map< std::string, AvahiServiceBrowser * >::iterator i;
  for (i = __browsers.begin(); i != __browsers.end(); ++i) {
    avahi_service_browser_free((*i).second);
  }
  __browsers.clear();
}


/** Call handler for a removed service.
 * @param name name
 * @param type type
 * @param domain domain
 */
void
AvahiThread::call_handler_service_removed( const char *name,
					    const char *type,
					    const char *domain)
{
  if ( __handlers.find(type) != __handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = __handlers[type].begin(); i != __handlers[type].end(); ++i) {
      (*i)->service_removed(name, type, domain);
    }
  }
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
AvahiThread::call_handler_service_added( const char *name,
					  const char *type,
					  const char *domain,
					  const char *host_name,
					  const AvahiAddress *address,
					  uint16_t port,
					  std::list<std::string> &txt,
					  AvahiLookupResultFlags flags)
{
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
  if ( __handlers.find(type) != __handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = __handlers[type].begin(); i != __handlers[type].end(); ++i) {
      (*i)->service_added(name, type, domain, host_name,
			  (struct sockaddr *)s, slen, port, txt, (int)flags);
    }
  }
  free(s);
}


/** Call handler for failure.
 * @param name name
 * @param type type
 * @param domain domain
 */
void
AvahiThread::call_handler_failed( const char *name,
				   const char *type,
				   const char *domain)
{
  if ( __handlers.find(type) != __handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = __handlers[type].begin(); i != __handlers[type].end(); ++i) {
      (*i)->browse_failed(name, type, domain);
    }
  }
}


/** Call handler "all for now".
 * @param type type
 */
void
AvahiThread::call_handler_all_for_now(const char *type)
{
  if ( __handlers.find(type) != __handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = __handlers[type].begin(); i != __handlers[type].end(); ++i) {
      (*i)->all_for_now();
    }
  }
}


/** Call handler "cache exhausted".
 * @param type type
 */
void
AvahiThread::call_handler_cache_exhausted(const char *type)
{
  if ( __handlers.find(type) != __handlers.end() ) {
    std::list<ServiceBrowseHandler *>::iterator i;
    for ( i = __handlers[type].begin(); i != __handlers[type].end(); ++i) {
      (*i)->cache_exhausted();
    }
  }
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
 * @param instance pointer to the AvahiThread instance that initiated
 * the search
 */
void
AvahiThread::browse_callback( AvahiServiceBrowser *b,
			      AvahiIfIndex interface,
			      AvahiProtocol protocol,
			      AvahiBrowserEvent event,
			      const char *name,
			      const char *type,
			      const char *domain,
			      AvahiLookupResultFlags flags,
			      void *instance)
{
  AvahiThread *at = static_cast<AvahiThread *>(instance);

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
    if (!(avahi_service_resolver_new(at->client, interface, protocol,
				     name, type, domain, protocol, (AvahiLookupFlags)0,
				     AvahiThread::resolve_callback, instance))) {
      throw NullPointerException("Could not instantiate resolver");
    }
    break;

  case AVAHI_BROWSER_REMOVE:
    // handler
    //printf("(Browser) REMOVE: service '%s' of type '%s' in domain '%s'\n", name, type, domain);
    at->call_handler_service_removed(name, type, domain);
    break;

  case AVAHI_BROWSER_ALL_FOR_NOW:
    // handler
    //printf("(Browser) ALL_FOR_NOW: service '%s' of type '%s' in domain '%s'\n", name, type, domain);
    at->call_handler_all_for_now(type);
    break;

  case AVAHI_BROWSER_CACHE_EXHAUSTED:
    // handler
    //printf("(Browser) CACHE_EXHAUSTED: service '%s' of type '%s' in domain '%s'\n", name, type, domain);
    at->call_handler_cache_exhausted(type);
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
 * @param instance pointer to the AvahiThread instance that initiated
 * the search
 */
void
AvahiThread::resolve_callback( AvahiServiceResolver *r,
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
  AvahiThread *at = static_cast<AvahiThread *>(instance);

  switch (event) {
  case AVAHI_RESOLVER_FAILURE:
    // handler failure
    at->call_handler_failed(name, type, domain);
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

      at->call_handler_service_added(name, type, domain, host_name, address, port, txts, flags);
    }
    break;
  }

  avahi_service_resolver_free(r);
}


/* **********************************************************************************
 * Avahi resolver methods
 * **********************************************************************************/


/** Order name resolution.
 * This initiates resolution of a name. The method immediately returns and will not
 * wait for the result.
 * @param name name to resolve.
 * @param handler handler to call for the result
 */
void
AvahiThread::resolve_name(const char *name, AvahiResolverHandler *handler)
{
  AvahiResolverCallbackData *data = new AvahiResolverCallbackData(this, handler);

  if ( __pending_hostname_resolves.find(name) == __pending_hostname_resolves.end()) {
    __pending_hostname_resolves[name] = data;
  }

  wake_poller();
}


void
AvahiThread::start_hostname_resolver(const char *name, AvahiResolverCallbackData *data)
{
  AvahiHostNameResolver *resolver;
  if ( (resolver = avahi_host_name_resolver_new(client, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC,
						name, AVAHI_PROTO_INET,
						AVAHI_LOOKUP_USE_MULTICAST,
						AvahiThread::host_name_resolver_callback,
						data) ) == NULL ) {
    throw Exception("Cannot create Avahi name resolver");
  } else {
    __running_hostname_resolvers.push_back(resolver);
  }

}


void
AvahiThread::start_hostname_resolvers()
{
  for (__phrit = __pending_hostname_resolves.begin(); __phrit != __pending_hostname_resolves.end(); ++__phrit) {
    start_hostname_resolver((*__phrit).first.c_str(), (*__phrit).second);
  }
  __pending_hostname_resolves.clear();
}


void
AvahiThread::start_address_resolvers()
{
  for (__parit = __pending_address_resolves.begin(); __parit != __pending_address_resolves.end(); ++__parit) {
    start_address_resolver((*__parit).first, (*__parit).second);
  }
  __pending_address_resolves.clear();
}


/** Order address resolution.
 * This initiates resolution of an address. The method immediately returns and will not
 * wait for the result.
 * @param addr address to resolve, currently only struct sockaddr_in is supported (IPv4)
 * @param addrlen length of addr in bytes
 * @param handler handler to call for the result
 */
void
AvahiThread::resolve_address(struct sockaddr *addr, socklen_t addrlen,
			     AvahiResolverHandler *handler)
{
  if ( addrlen != sizeof(struct sockaddr_in) ) {
    throw Exception("Only IPv4 is currently supported");
  }

  struct sockaddr_in *in_addr = (struct sockaddr_in *)calloc(1, sizeof(struct sockaddr_in));
  memcpy(in_addr, addr, sizeof(struct sockaddr_in));
  AvahiResolverCallbackData *data = new AvahiResolverCallbackData(this, handler);

  __pending_address_resolves[in_addr] = data;
  wake_poller();
}


void
AvahiThread::start_address_resolver(struct sockaddr_in *in_addr, AvahiResolverCallbackData *data)
{
  AvahiAddress a;
  a.proto = AVAHI_PROTO_INET;
  a.data.ipv4.address = in_addr->sin_addr.s_addr;

  AvahiAddressResolver *resolver;
  if ( (resolver = avahi_address_resolver_new(client, AVAHI_IF_UNSPEC, AVAHI_PROTO_UNSPEC,
					      &a, AVAHI_LOOKUP_USE_MULTICAST,
					      AvahiThread::address_resolver_callback,
					      data) ) == NULL ) {
    Exception e("Cannot create Avahi address resolver");
    e.append("Avahi error: %s", avahi_strerror(avahi_client_errno(client)));
    throw e;
  } else {
    __running_address_resolvers.push_back_locked(resolver);
  }
}


/** Remove hostname resolver.
 * Used internally by callback.
 * @param r resolver
 */
void
AvahiThread::remove_hostname_resolver(AvahiHostNameResolver *r)
{
  __running_hostname_resolvers.remove_locked(r);
}


/** Remove address resolver.
 * Used internally by callback.
 * @param r resolver
 */
void
AvahiThread::remove_address_resolver(AvahiAddressResolver *r)
{
  __running_address_resolvers.remove_locked(r);
}


/** Internal callback.
 * Callback for avahi.
 */
void
AvahiThread::host_name_resolver_callback(AvahiHostNameResolver *r,
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
      res->sin_family = (unsigned short)avahi_proto_to_af(protocol);
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
AvahiThread::address_resolver_callback(AvahiAddressResolver *r,
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
  res->sin_family = (unsigned short)avahi_proto_to_af(protocol);
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


/** Unlocks init lock.
 * Only to be called by client_callback().
 */
void
AvahiThread::init_done()
{
  wake_poller();
  init_wc->wake_all();
}


/** Waits for the AvahiThread to be initialized.
 * You can use this if you want to wait until the thread has been
 * fully initialized and may be used. Since the happens in this thread
 * it is in general not immediately ready after start().
 * This will block the calling thread until the AvahiThread has
 * been initialized. This is done by waiting for a release of an
 * initialization mutex.
 */
void
AvahiThread::wait_initialized()
{
  init_wc->wait();
}

} // end namespace fawkes
