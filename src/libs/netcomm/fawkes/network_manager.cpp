
/***************************************************************************
 *  network_manager.cpp - Fawkes network manager
 *
 *  Created: Wed Nov 16 00:05:18 2006
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

#include <netcomm/fawkes/network_manager.h>
#include <core/threading/thread_collector.h>

#include <core/exceptions/system.h>
#include <netcomm/fawkes/server_thread.h>
#include <netcomm/fawkes/handler.h>
#include <netcomm/utils/resolver.h>
#ifdef HAVE_AVAHI
#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/service_discovery/service.h>
#else
#include <netcomm/service_discovery/dummy_service_publisher.h>
#include <netcomm/service_discovery/dummy_service_browser.h>
#endif

#include <cstdlib>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class FawkesNetworkManager <netcomm/fawkes/network_manager.h>
 * Fawkes Network Manager.
 * This class provides a manager for network connections used in Fawkes.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param thread_collector thread collector that threads shall be registered to
 * @param enable_ipv4 true to listen on the IPv4 TCP port
 * @param enable_ipv6 true to listen on the IPv6 TCP port
 * @param listen_ipv4 IPv4 address to listen on for incoming connections,
 * empty string or 0.0.0.0 to listen on any local address
 * @param listen_ipv6 IPv6 address to listen on for incoming connections,
 * empty string or :: to listen on any local address
 * @param fawkes_port port to listen on for Fawkes network connections
 * @param service_name Avahi service name for Fawkes network service
 */
FawkesNetworkManager::FawkesNetworkManager(ThreadCollector *thread_collector,
                                           bool enable_ipv4, bool enable_ipv6,
                                           const std::string &listen_ipv4, const std::string &listen_ipv6,
                                           unsigned short int fawkes_port,
                                           const char *service_name)
{
  __fawkes_port      = fawkes_port;
  __thread_collector = thread_collector;
  __fawkes_network_thread = new FawkesNetworkServerThread(enable_ipv4, enable_ipv6,
                                                          listen_ipv4, listen_ipv6,
                                                          __fawkes_port,
                                                          __thread_collector);
  __thread_collector->add(__fawkes_network_thread);
#ifdef HAVE_AVAHI
  __avahi_thread          = new AvahiThread(enable_ipv4, enable_ipv6);
  __service_publisher     = __avahi_thread;
  __service_browser       = __avahi_thread;
  __thread_collector->add(__avahi_thread);
  __nnresolver = new NetworkNameResolver(__avahi_thread);
  NetworkService *fawkes_service = new NetworkService(__nnresolver, service_name,
						      "_fawkes._tcp",
						      __fawkes_port);
  __avahi_thread->publish_service(fawkes_service);
  delete fawkes_service;
#else
  __service_publisher = new DummyServicePublisher();
  __service_browser   = new DummyServiceBrowser();
  __nnresolver        = new NetworkNameResolver();
#endif
}


/** Destructor. */
FawkesNetworkManager::~FawkesNetworkManager()
{
  __thread_collector->remove(__fawkes_network_thread);
  delete __fawkes_network_thread;
#ifdef HAVE_AVAHI
  __thread_collector->remove(__avahi_thread);
  delete __avahi_thread;
#else
  delete __service_publisher;
  delete __service_browser;
#endif
  delete __nnresolver;
}


/** Get Fawkes network hub.
 * @return Fawkes network hub
 */
FawkesNetworkHub *
FawkesNetworkManager::hub()
{
  return __fawkes_network_thread;
}


/** Get network name resolver.
 * @return network name resolver
 */
NetworkNameResolver *
FawkesNetworkManager::nnresolver()
{
  return __nnresolver;
}


/** Get service publisher
 * @return service publisher
 */
ServicePublisher *
FawkesNetworkManager::service_publisher()
{
  return __service_publisher;
}


/** Get service browser.
 * @return service browser
 */
ServiceBrowser *
FawkesNetworkManager::service_browser()
{
  return __service_browser;
}

/** Get Fawkes TCP port.
 * @return TCP port on which Fawkes is listening
 */
unsigned short int
FawkesNetworkManager::fawkes_port() const
{
  return __fawkes_port;
}


} // end namespace fawkes
