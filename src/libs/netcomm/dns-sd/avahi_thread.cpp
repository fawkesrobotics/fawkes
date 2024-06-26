
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

#include <arpa/inet.h>
#include <avahi-client/lookup.h>
#include <avahi-client/publish.h>
#include <avahi-common/alternative.h>
#include <avahi-common/error.h>
#include <avahi-common/malloc.h>
#include <avahi-common/simple-watch.h>
#include <avahi-common/timeval.h>
#include <core/exceptions/software.h>
#include <core/threading/mutex.h>
#include <core/threading/wait_condition.h>
#include <net/if.h>
#include <netcomm/dns-sd/avahi_resolver_handler.h>
#include <netcomm/dns-sd/avahi_thread.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <utils/misc/string_conversions.h>

#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netdb.h>
#include <thread>

namespace fawkes {

/** @class AvahiThread netcomm/dns-sd/avahi_thread.h
 * Avahi main thread.
 * This thread handles all tasks related to avahi. This is the single
 * interaction point with the Avahi adapter.
 *
 * @ingroup NetComm
 * @author Tim Niemueller
 */

/** Time to wait if creating an avahi client fails. **/
const std::chrono::seconds AvahiThread::wait_on_init_failure{5};

/** Constructor.
 * You can choose whether to announce IPv4 or IPv6 only or both.
 * If you select both, new service will be created with the "unspecified"
 * address family in Avahi, causing it to announce the service on all
 * supported protocols (which may or may not include both).
 * @param enable_ipv4 enable IPv4 support
 * @param enable_ipv6 enable IPv6 support
 */
AvahiThread::AvahiThread(bool enable_ipv4, bool enable_ipv6)
: Thread("AvahiThread"), enable_ipv4(enable_ipv4), enable_ipv6(enable_ipv6)
{
	simple_poll = NULL;
	client      = NULL;

	need_recover    = false;
	do_reset_groups = false;

	if (enable_ipv4 && enable_ipv6) {
		service_protocol = AVAHI_PROTO_UNSPEC;
	} else if (enable_ipv4) {
		service_protocol = AVAHI_PROTO_INET;
	} else if (enable_ipv6) {
		service_protocol = AVAHI_PROTO_INET6;
	} else {
		throw Exception("Neither IPv4 nor IPv6 enabled");
	}

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

	if (client)
		avahi_client_free(client);

	if (simple_poll)
		avahi_simple_poll_free(simple_poll);
}

/** Avahi thread loop.
 * The avahi thread calls the simple poll iterate to poll with an infinite
 * timeout. This way the loop blocks until an event occurs.
 */
void
AvahiThread::loop()
{
	if (need_recover) {
		if (client) {
			avahi_client_free(client);
			client = NULL;
		}

		if (simple_poll) {
			avahi_simple_poll_free(simple_poll);
			simple_poll = NULL;
		}
	}

	if (!simple_poll) {
		// Init
		int error;

		if ((simple_poll = avahi_simple_poll_new())) {
			client = avahi_client_new(avahi_simple_poll_get(simple_poll),
			                          AVAHI_CLIENT_NO_FAIL,
			                          AvahiThread::client_callback,
			                          this,
			                          &error);

			if (!client) {
				avahi_simple_poll_free(simple_poll);
				simple_poll = NULL;
			}
		}
	}

	if (client) {
		if (do_reset_groups) {
			reset_groups();
			recreate_services();
		}
		if (need_recover) {
			erase_groups();
			erase_browsers();
			recreate_services();
			recreate_browsers();
		}
		if (client_state == AVAHI_CLIENT_S_RUNNING) {
			remove_pending_services();
			remove_pending_browsers();
			create_pending_services();
			create_pending_browsers();
			start_hostname_resolvers();
			start_address_resolvers();
		}

		need_recover = false;

		avahi_simple_poll_iterate(simple_poll, -1);
	} else {
		// We failed to create a client, e.g., because the daemon is not running.
		// Wait for a while and try again.
		std::this_thread::sleep_for(wait_on_init_failure);
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
	if (simple_poll) {
		avahi_simple_poll_wakeup(simple_poll);
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
	AvahiThread *at  = static_cast<AvahiThread *>(instance);
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
	if (services_.find(service) == services_.end()) {
		pending_services_.push_locked(service);
	} else {
		throw Exception("Service already registered");
	}

	wake_poller();
}

void
AvahiThread::unpublish_service(NetworkService *service)
{
	if (services_.find(*service) != services_.end()) {
		pending_remove_services_.push_locked(service);
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
	if (!client)
		return NULL;

	AvahiEntryGroup *group;
	if (exgroup) {
		group = exgroup;
	} else {
		if (!(group = avahi_entry_group_new(client, AvahiThread::entry_group_callback, this))) {
			throw NullPointerException("Cannot create service group");
		}
	}

	AvahiStringList              *al = NULL;
	const std::list<std::string> &l  = service.txt();
	for (std::list<std::string>::const_iterator j = l.begin(); j != l.end(); ++j) {
		al = avahi_string_list_add(al, j->c_str());
	}

	int         rv   = AVAHI_ERR_COLLISION;
	std::string name = service.modified_name() ? service.modified_name() : service.name();
	for (int i = 1; (i <= 100) && (rv == AVAHI_ERR_COLLISION); ++i) {
		rv = avahi_entry_group_add_service_strlst(group,
		                                          AVAHI_IF_UNSPEC,
		                                          service_protocol,
		                                          (AvahiPublishFlags)0,
		                                          name.c_str(),
		                                          service.type(),
		                                          service.domain(),
		                                          service.host(),
		                                          service.port(),
		                                          al);

		if (rv == AVAHI_ERR_COLLISION) {
			char *n = avahi_alternative_service_name(name.c_str());
			service.set_modified_name(n);
			name = n;
			avahi_free(n);
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
	for (sit_ = services_.begin(); sit_ != services_.end(); ++sit_) {
		(*sit_).second = create_service(sit_->first, sit_->second);
	}
}

void
AvahiThread::create_pending_services()
{
	pending_services_.lock();
	while (!pending_services_.empty()) {
		NetworkService &s = pending_services_.front();
		services_[s]      = create_service(s, NULL);
		pending_services_.pop();
	}
	pending_services_.unlock();
}

void
AvahiThread::remove_pending_services()
{
	Thread::CancelState old_state;
	set_cancel_state(CANCEL_DISABLED, &old_state);
	pending_remove_services_.lock();
	while (!pending_remove_services_.empty()) {
		NetworkService &s = pending_remove_services_.front();
		if (services_.find(s) != services_.end()) {
			group_erase(services_[s]);
			services_.erase_locked(s);
		}
		pending_remove_services_.pop();
	}
	pending_remove_services_.unlock();
	set_cancel_state(old_state);
}

/** Drop our registered services.
 * When the server is back in AVAHI_SERVER_RUNNING state we will register them
 * again with the new host name (triggered by AvahiThread).
 */
void
AvahiThread::group_reset(AvahiEntryGroup *g)
{
	if (g) {
		avahi_entry_group_reset(g);
	}
}

/** Erase service group. */
void
AvahiThread::group_erase(AvahiEntryGroup *g)
{
	if (g) {
		avahi_entry_group_reset(g);
		avahi_entry_group_free(g);
	}
}

void
AvahiThread::erase_groups()
{
	for (sit_ = services_.begin(); sit_ != services_.end(); ++sit_) {
		if (sit_->second)
			group_erase(sit_->second);
		sit_->second = NULL;
	}
}

void
AvahiThread::reset_groups()
{
	for (sit_ = services_.begin(); sit_ != services_.end(); ++sit_) {
		group_reset((*sit_).second);
	}
}

/** Called if there was a name collision. */
void
AvahiThread::name_collision(AvahiEntryGroup *g)
{
	for (sit_ = services_.begin(); sit_ != services_.end(); ++sit_) {
		if ((*sit_).second == g) {
			NetworkService service = sit_->first;
			std::string    name    = service.modified_name() ? service.modified_name() : service.name();

			/* A service name collision happened. Let's pick a new name */
			char *n = avahi_alternative_service_name((*sit_).first.name());
			service.set_modified_name(n);
			avahi_free(n);

			pending_remove_services_.push_locked(service);
			pending_services_.push_locked(service);
		}
	}
}

/** Callback for Avahi.
 * @param g entry group
 * @param state new state
 * @param instance instance of AvahiThread that triggered the event.
 */
void
AvahiThread::entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state, void *instance)
{
	AvahiThread *at = static_cast<AvahiThread *>(instance);

	switch (state) {
	case AVAHI_ENTRY_GROUP_ESTABLISHED:
		/* The entry group has been established successfully */
		//fprintf(stderr, "Service '%s' successfully established.\n", name);
		break;

	case AVAHI_ENTRY_GROUP_COLLISION: {
		at->name_collision(g);
		break;
	}

	case AVAHI_ENTRY_GROUP_FAILURE:
		/* Some kind of failure happened while we were registering our services */
		at->recover();
		break;

	case AVAHI_ENTRY_GROUP_UNCOMMITED:
	case AVAHI_ENTRY_GROUP_REGISTERING: break;
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
	handlers_[service_type].push_back(h);
	pending_browsers_.push_locked(service_type);

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
	if (handlers_.find(service_type) != handlers_.end()) {
		handlers_[service_type].remove(h);
		if (handlers_[service_type].size() == 0) {
			if (browsers_.find(service_type) != browsers_.end()) {
				pending_browser_removes_.push_locked(service_type);
				//avahi_service_browser_free(browsers_[service_type]);
				//browsers_.erase(service_type);
			}
			handlers_.erase(service_type);
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
	if (browsers_.find(service_type) == browsers_.end()) {
		if (client) {
			AvahiServiceBrowser *b = avahi_service_browser_new(client,
			                                                   AVAHI_IF_UNSPEC,
			                                                   service_protocol,
			                                                   service_type,
			                                                   NULL,
			                                                   (AvahiLookupFlags)0,
			                                                   AvahiThread::browse_callback,
			                                                   this);

			if (!b) {
				handlers_[service_type].pop_back();
				throw NullPointerException("Could not instantiate AvahiServiceBrowser");
			}
			browsers_[service_type] = b;
		}
	}
}

/** Create browsers.
 * Creates browser for all services.
 */
void
AvahiThread::recreate_browsers()
{
	LockMap<std::string, std::list<ServiceBrowseHandler *>>::iterator i;
	for (i = handlers_.begin(); i != handlers_.end(); ++i) {
		create_browser((*i).first.c_str());
	}
}

void
AvahiThread::create_pending_browsers()
{
	pending_browsers_.lock();
	while (!pending_browsers_.empty()) {
		//printf("Creating browser for %s\n", pending_browsers_.front().c_str());
		create_browser(pending_browsers_.front().c_str());
		pending_browsers_.pop();
	}
	pending_browsers_.unlock();
}

void
AvahiThread::remove_pending_browsers()
{
	Thread::CancelState old_state;
	set_cancel_state(CANCEL_DISABLED, &old_state);
	pending_browser_removes_.lock();
	while (!pending_browser_removes_.empty()) {
		std::string &s = pending_browser_removes_.front();
		avahi_service_browser_free(browsers_[s]);
		browsers_.erase_locked(s);
		pending_browser_removes_.pop();
	}
	pending_browser_removes_.unlock();
	set_cancel_state(old_state);
}

/** Erase all browsers. */
void
AvahiThread::erase_browsers()
{
	std::map<std::string, AvahiServiceBrowser *>::iterator i;
	for (i = browsers_.begin(); i != browsers_.end(); ++i) {
		avahi_service_browser_free((*i).second);
	}
	browsers_.clear();
}

/** Call handler for a removed service.
 * @param name name
 * @param type type
 * @param domain domain
 */
void
AvahiThread::call_handler_service_removed(const char *name, const char *type, const char *domain)
{
	if (handlers_.find(type) != handlers_.end()) {
		std::list<ServiceBrowseHandler *>::iterator i;
		for (i = handlers_[type].begin(); i != handlers_[type].end(); ++i) {
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
AvahiThread::call_handler_service_added(const char             *name,
                                        const char             *type,
                                        const char             *domain,
                                        const char             *host_name,
                                        const AvahiIfIndex      interface,
                                        const AvahiAddress     *address,
                                        uint16_t                port,
                                        std::list<std::string> &txt,
                                        AvahiLookupResultFlags  flags)
{
	char ifname[IF_NAMESIZE];
	ifname[0] = 0;
	if (if_indextoname(interface, ifname) == NULL) {
		fprintf(stderr, "AvahiThread::call_handler_service_added: IPv6 if_indextoname failed");
		return;
	}

	struct sockaddr *s = NULL;
	socklen_t        slen;
	if (address->proto == AVAHI_PROTO_INET) {
		if (!enable_ipv4)
			return;
		slen                    = sizeof(struct sockaddr_in);
		struct sockaddr_in *sin = (struct sockaddr_in *)malloc(slen);
		sin->sin_family         = AF_INET;
		sin->sin_addr.s_addr    = address->data.ipv4.address;
		sin->sin_port           = htons(port);
		s                       = (struct sockaddr *)sin;
	} else if (address->proto == AVAHI_PROTO_INET6) {
		if (!enable_ipv6)
			return;
		slen                     = sizeof(struct sockaddr_in6);
		struct sockaddr_in6 *sin = (struct sockaddr_in6 *)malloc(slen);
		sin->sin6_family         = AF_INET6;
		memcpy(&sin->sin6_addr, &address->data.ipv6.address, sizeof(in6_addr));

		char ipaddr[INET6_ADDRSTRLEN];
		if (inet_ntop(AF_INET6, &sin->sin6_addr, ipaddr, sizeof(ipaddr)) != NULL) {
			std::string addr_with_scope = std::string(ipaddr) + "%" + ifname;
			std::string port_s          = StringConversions::to_string((unsigned int)port);

			// use getaddrinfo to fill especially to determine scope ID
			struct addrinfo hints, *res;
			memset(&hints, 0, sizeof(hints));
			hints.ai_family = AF_INET6;
			hints.ai_flags  = AI_NUMERICHOST;
			if (getaddrinfo(addr_with_scope.c_str(), port_s.c_str(), &hints, &res) == 0) {
				if (slen == res[0].ai_addrlen) {
					memcpy(sin, res[0].ai_addr, slen);
					freeaddrinfo(res);
				} else {
					fprintf(stderr,
					        "AvahiThread::call_handler_service_added: IPv6 address lengths different");
					freeaddrinfo(res);
					return;
				}
			} else {
				fprintf(stderr, "AvahiThread::call_handler_service_added: IPv6 getaddrinfo failed");
				return;
			}
		} else {
			fprintf(stderr, "AvahiThread::call_handler_service_added: IPv6 inet_ntop failed");
			return;
		}
		s = (struct sockaddr *)sin;
	} else {
		// ignore
		return;
	}
	if (handlers_.find(type) != handlers_.end()) {
		std::list<ServiceBrowseHandler *>::iterator i;
		for (i = handlers_[type].begin(); i != handlers_[type].end(); ++i) {
			(*i)->service_added(
			  name, type, domain, host_name, ifname, (struct sockaddr *)s, slen, port, txt, (int)flags);
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
AvahiThread::call_handler_failed(const char *name, const char *type, const char *domain)
{
	if (handlers_.find(type) != handlers_.end()) {
		std::list<ServiceBrowseHandler *>::iterator i;
		for (i = handlers_[type].begin(); i != handlers_[type].end(); ++i) {
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
	if (handlers_.find(type) != handlers_.end()) {
		std::list<ServiceBrowseHandler *>::iterator i;
		for (i = handlers_[type].begin(); i != handlers_[type].end(); ++i) {
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
	if (handlers_.find(type) != handlers_.end()) {
		std::list<ServiceBrowseHandler *>::iterator i;
		for (i = handlers_[type].begin(); i != handlers_[type].end(); ++i) {
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
AvahiThread::browse_callback(AvahiServiceBrowser   *b,
                             AvahiIfIndex           interface,
                             AvahiProtocol          protocol,
                             AvahiBrowserEvent      event,
                             const char            *name,
                             const char            *type,
                             const char            *domain,
                             AvahiLookupResultFlags flags,
                             void                  *instance)
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
		if (!(avahi_service_resolver_new(at->client,
		                                 interface,
		                                 protocol,
		                                 name,
		                                 type,
		                                 domain,
		                                 protocol,
		                                 (AvahiLookupFlags)0,
		                                 AvahiThread::resolve_callback,
		                                 instance))) {
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
AvahiThread::resolve_callback(AvahiServiceResolver          *r,
                              AvahiIfIndex                   interface,
                              AVAHI_GCC_UNUSED AvahiProtocol protocol,
                              AvahiResolverEvent             event,
                              const char                    *name,
                              const char                    *type,
                              const char                    *domain,
                              const char                    *host_name,
                              const AvahiAddress            *address,
                              uint16_t                       port,
                              AvahiStringList               *txt,
                              AvahiLookupResultFlags         flags,
                              void                          *instance)
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
			std::list<std::string> txts;
			AvahiStringList       *l = txt;

			txts.clear();
			while (l) {
				txts.push_back((char *)avahi_string_list_get_text(l));
				l = avahi_string_list_get_next(l);
			}

			at->call_handler_service_added(
			  name, type, domain, host_name, interface, address, port, txts, flags);
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

	if (pending_hostname_resolves_.find(name) == pending_hostname_resolves_.end()) {
		pending_hostname_resolves_[name] = data;
	}

	wake_poller();
}

void
AvahiThread::start_hostname_resolver(const char *name, AvahiResolverCallbackData *data)
{
	AvahiHostNameResolver *resolver;
	if ((resolver = avahi_host_name_resolver_new(client,
	                                             AVAHI_IF_UNSPEC,
	                                             AVAHI_PROTO_UNSPEC,
	                                             name,
	                                             service_protocol,
	                                             AVAHI_LOOKUP_USE_MULTICAST,
	                                             AvahiThread::host_name_resolver_callback,
	                                             data))
	    == NULL) {
		throw Exception("Cannot create Avahi name resolver");
	} else {
		running_hostname_resolvers_.push_back(resolver);
	}
}

void
AvahiThread::start_hostname_resolvers()
{
	LockMap<std::string, AvahiResolverCallbackData *>::iterator phrit;
	for (phrit = pending_hostname_resolves_.begin(); phrit != pending_hostname_resolves_.end();
	     ++phrit) {
		start_hostname_resolver(phrit->first.c_str(), phrit->second);
	}
	pending_hostname_resolves_.clear();
}

void
AvahiThread::start_address_resolvers()
{
	LockMap<struct ::sockaddr_storage *, AvahiResolverCallbackData *>::iterator parit;

	for (parit = pending_address_resolves_.begin(); parit != pending_address_resolves_.end();
	     ++parit) {
		start_address_resolver(parit->first, parit->second);
		free(parit->first);
	}
	pending_address_resolves_.clear();
}

/** Order address resolution.
 * This initiates resolution of an address. The method immediately returns and will not
 * wait for the result.
 * @param addr address to resolve
 * @param addrlen length of addr in bytes
 * @param handler handler to call for the result
 */
void
AvahiThread::resolve_address(struct sockaddr      *addr,
                             socklen_t             addrlen,
                             AvahiResolverHandler *handler)
{
	struct ::sockaddr_storage *sstor =
	  (struct ::sockaddr_storage *)malloc(sizeof(struct ::sockaddr_storage));
	if (addr->sa_family == AF_INET) {
		if (addrlen != sizeof(sockaddr_in)) {
			throw Exception("Invalid size for IPv4 address struct");
		}
		memcpy(sstor, addr, sizeof(sockaddr_in));
	} else if (addr->sa_family == AF_INET6) {
		if (addrlen != sizeof(sockaddr_in6)) {
			throw Exception("Invalid size for IPv6 address struct");
		}
		memcpy(sstor, addr, sizeof(sockaddr_in6));
	} else {
		throw Exception("Unknown address family");
	}
	AvahiResolverCallbackData *data = new AvahiResolverCallbackData(this, handler);

	pending_address_resolves_[sstor] = data;
	wake_poller();
}

void
AvahiThread::start_address_resolver(const struct sockaddr_storage *in_addr,
                                    AvahiResolverCallbackData     *data)
{
	AvahiAddress a;

	if (in_addr->ss_family == AF_INET) {
		a.proto             = AVAHI_PROTO_INET;
		a.data.ipv4.address = ((sockaddr_in *)in_addr)->sin_addr.s_addr;
	} else if (in_addr->ss_family == AF_INET6) {
		a.proto = AVAHI_PROTO_INET6;
		memcpy(&a.data.ipv6.address, &((sockaddr_in6 *)in_addr)->sin6_addr, sizeof(in6_addr));
	} else {
		throw Exception("Unknown address family");
	}

	AvahiAddressResolver *resolver;
	if ((resolver = avahi_address_resolver_new(client,
	                                           AVAHI_IF_UNSPEC,
	                                           AVAHI_PROTO_UNSPEC,
	                                           &a,
	                                           AVAHI_LOOKUP_USE_MULTICAST,
	                                           AvahiThread::address_resolver_callback,
	                                           data))
	    == NULL) {
		Exception e("Cannot create Avahi address resolver");
		e.append("Avahi error: %s", avahi_strerror(avahi_client_errno(client)));
		throw e;
	} else {
		running_address_resolvers_.push_back_locked(resolver);
	}
}

/** Remove hostname resolver.
 * Used internally by callback.
 * @param r resolver
 */
void
AvahiThread::remove_hostname_resolver(AvahiHostNameResolver *r)
{
	running_hostname_resolvers_.remove_locked(r);
}

/** Remove address resolver.
 * Used internally by callback.
 * @param r resolver
 */
void
AvahiThread::remove_address_resolver(AvahiAddressResolver *r)
{
	running_address_resolvers_.remove_locked(r);
}

/** Internal callback.
 * Callback for avahi.
 */
void
AvahiThread::host_name_resolver_callback(AvahiHostNameResolver *r,
                                         AvahiIfIndex           interface,
                                         AvahiProtocol          protocol,
                                         AvahiResolverEvent     event,
                                         const char            *name,
                                         const AvahiAddress    *a,
                                         AvahiLookupResultFlags flags,
                                         void                  *userdata)
{
	AvahiResolverCallbackData *cd = static_cast<AvahiResolverCallbackData *>(userdata);

	cd->first->remove_hostname_resolver(r);
	avahi_host_name_resolver_free(r);

	switch (event) {
	case AVAHI_RESOLVER_FOUND: {
		if (protocol == AVAHI_PROTO_INET) {
			struct sockaddr_in *res = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
			res->sin_family         = (unsigned short)avahi_proto_to_af(protocol);
			res->sin_addr.s_addr    = a->data.ipv4.address;
			cd->second->resolved_name(strdup(name), (struct sockaddr *)res, sizeof(struct sockaddr_in));
		} else if (protocol == AVAHI_PROTO_INET6) {
			struct sockaddr_in6 *res = (struct sockaddr_in6 *)malloc(sizeof(struct sockaddr_in6));
			res->sin6_family         = (unsigned short)avahi_proto_to_af(protocol);
			memcpy(&res->sin6_addr, &a->data.ipv6.address, sizeof(in6_addr));
			cd->second->resolved_name(strdup(name), (struct sockaddr *)res, sizeof(struct sockaddr_in6));
		} else { // don't know
			cd->second->name_resolution_failed(strdup(name));
		}
	} break;

	case AVAHI_RESOLVER_FAILURE:
	default: cd->second->name_resolution_failed(strdup(name)); break;
	}

	delete cd;
}

/** Internal callback.
 * Callback for avahi.
 */
void
AvahiThread::address_resolver_callback(AvahiAddressResolver  *r,
                                       AvahiIfIndex           interface,
                                       AvahiProtocol          protocol,
                                       AvahiResolverEvent     event,
                                       const AvahiAddress    *a,
                                       const char            *name,
                                       AvahiLookupResultFlags flags,
                                       void                  *userdata)
{
	AvahiResolverCallbackData *cd = static_cast<AvahiResolverCallbackData *>(userdata);

	cd->first->remove_address_resolver(r);
	avahi_address_resolver_free(r);

	struct sockaddr *res      = NULL;
	socklen_t        res_size = 0;

	if (protocol == AVAHI_PROTO_INET) {
		res_size               = sizeof(struct sockaddr_in);
		res                    = (struct sockaddr *)malloc(res_size);
		sockaddr_in *res_4     = (struct sockaddr_in *)res;
		res_4->sin_family      = (unsigned short)avahi_proto_to_af(protocol);
		res_4->sin_addr.s_addr = a->data.ipv4.address;
	} else if (protocol == AVAHI_PROTO_INET6) {
		res_size            = sizeof(struct sockaddr_in6);
		res                 = (struct sockaddr *)malloc(res_size);
		sockaddr_in6 *res_6 = (struct sockaddr_in6 *)res;
		res_6->sin6_family  = (unsigned short)avahi_proto_to_af(protocol);
		memcpy(&res_6->sin6_addr, &a->data.ipv6.address, sizeof(in6_addr));
	}

	switch (event) {
	case AVAHI_RESOLVER_FOUND: cd->second->resolved_address(res, res_size, strdup(name)); break;
	case AVAHI_RESOLVER_FAILURE: cd->second->address_resolution_failed(res, res_size); break;

	default: cd->second->address_resolution_failed(NULL, 0); break;
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
