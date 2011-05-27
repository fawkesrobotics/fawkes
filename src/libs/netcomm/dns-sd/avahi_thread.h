
/***************************************************************************
 *  avahi_thread.h - Avahi Thread
 *
 *  Created: Wed Nov 08 11:17:06 2006
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

#ifndef __NETCOMM_DNSSD_AVAHI_THREAD_H_
#define __NETCOMM_DNSSD_AVAHI_THREAD_H_

#include <core/threading/thread.h>
#include <netcomm/service_discovery/service_publisher.h>
#include <netcomm/service_discovery/service_browser.h>

#include <core/utils/lock_map.h>
#include <core/utils/lock_list.h>
#include <core/utils/lock_queue.h>

#include <avahi-client/client.h>
#include <string>
#include <utility>
#include <netinet/in.h>

struct AvahiEntryGroup;
struct AvahiSimplePoll;
struct AvahiServiceBrowser;
struct AvahiServiceResolver;
struct AvahiHostNameResolver;
struct AvahiAddressResolver;
struct sockaddr_in;

namespace fawkes {

class ServiceBrowseHandler;
class NetworkService;
class WaitCondition;
class AvahiResolverHandler;

class AvahiThread
: public Thread,
  public ServicePublisher,
  public ServiceBrowser
{
 public:
  AvahiThread();
  ~AvahiThread();

  void wait_initialized();

  virtual void loop();

  /* Service publisher entry methods */
  void publish_service(NetworkService *service);
  void unpublish_service(NetworkService *service);

  /* Service browser methods */
  void watch_service(const char *service_type, ServiceBrowseHandler *h);
  void unwatch_service(const char *service_type, ServiceBrowseHandler *h);

  /* Resolver methods */
  void resolve_name(const char *name, AvahiResolverHandler *handler);
  void resolve_address(struct sockaddr *addr, socklen_t addrlen,
		       AvahiResolverHandler *handler);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  /* Callbacks */
  static void client_callback(AvahiClient *c, AvahiClientState state, void *instance);

  static void entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state,
				   void *instance);

  static void browse_callback( AvahiServiceBrowser *b,
			       AvahiIfIndex interface,
			       AvahiProtocol protocol,
			       AvahiBrowserEvent event,
			       const char *name,
			       const char *type,
			       const char *domain,
			       AvahiLookupResultFlags flags,
			       void *instance);

  static void resolve_callback( AvahiServiceResolver *r,
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
				void *instance);

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


  void call_handler_service_removed( const char *name,
				     const char *type,
				     const char *domain);
  void call_handler_service_added( const char *name,
				   const char *type,
				   const char *domain,
				   const char *host_name,
				   const AvahiAddress *address,
				   uint16_t port,
				   std::list<std::string> &txt,
				   AvahiLookupResultFlags flags);
  void call_handler_failed( const char *name,
			    const char *type,
			    const char *domain);

  void call_handler_all_for_now(const char *type);
  void call_handler_cache_exhausted(const char *type);


  void create_browser(const char *service_type);
  void create_browsers();
  void erase_browsers();
  void recreate_browsers();
  void create_pending_browsers();
  void remove_pending_browsers();

  /* general private methods */
  void init_done();
  void recover();
  void wake_poller();

  /* publisher private methods */
  AvahiEntryGroup * create_service(const NetworkService &service,
				   AvahiEntryGroup *exgroup);
  void group_reset(AvahiEntryGroup *g);
  void group_erase(AvahiEntryGroup *g);
  void name_collision(AvahiEntryGroup *g);
  void erase_groups();
  void reset_groups();
  void create_pending_services();
  void remove_pending_services();
  void recreate_services();

  /* resolver */
  /** Internal type to pass data to callbacks for resolve methods */
  typedef std::pair<AvahiThread *, AvahiResolverHandler *> AvahiResolverCallbackData;

  void remove_hostname_resolver(AvahiHostNameResolver *r);
  void remove_address_resolver(AvahiAddressResolver *r);
  void start_address_resolvers();
  void start_hostname_resolvers();
  void start_hostname_resolver(const char *name, AvahiResolverCallbackData *data);
  void start_address_resolver(struct sockaddr_in *in_addr, AvahiResolverCallbackData *data);


  bool need_recover;
  bool do_erase_browsers;
  bool do_reset_groups;

  AvahiSimplePoll  *simple_poll;
  AvahiClient      *client;
  AvahiClientState  client_state;

  WaitCondition         *init_wc;

  LockMap<NetworkService, AvahiEntryGroup *>           __services;
  LockMap<NetworkService, AvahiEntryGroup *>::iterator __sit;
  LockQueue<NetworkService>            __pending_services;
  LockQueue<NetworkService>            __pending_remove_services;

  LockMap<std::string, std::list<ServiceBrowseHandler *> > __handlers;
  LockMap<std::string, AvahiServiceBrowser * >             __browsers;
  LockQueue<std::string> __pending_browsers;
  LockQueue<std::string> __pending_browser_removes;

  LockList<AvahiHostNameResolver *> __running_hostname_resolvers;
  LockList<AvahiAddressResolver *>  __running_address_resolvers;

  LockMap<std::string, AvahiResolverCallbackData * >      __pending_hostname_resolves;
  LockMap<std::string, AvahiResolverCallbackData * >::iterator __phrit;
  LockMap<struct sockaddr_in *, AvahiResolverCallbackData *>  __pending_address_resolves;
  LockMap<struct sockaddr_in *, AvahiResolverCallbackData *>::iterator  __parit;
};

} // end namespace fawkes

#endif
