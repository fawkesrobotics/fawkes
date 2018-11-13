
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
  fawkes_port_      = fawkes_port;
  thread_collector_ = thread_collector;
  fawkes_network_thread_ = new FawkesNetworkServerThread(enable_ipv4, enable_ipv6,
                                                          listen_ipv4, listen_ipv6,
                                                          fawkes_port_,
                                                          thread_collector_);
  thread_collector_->add(fawkes_network_thread_);
#ifdef HAVE_AVAHI
  avahi_thread_          = new AvahiThread(enable_ipv4, enable_ipv6);
  service_publisher_     = avahi_thread_;
  service_browser_       = avahi_thread_;
  thread_collector_->add(avahi_thread_);
  nnresolver_ = new NetworkNameResolver(avahi_thread_);
  NetworkService *fawkes_service = new NetworkService(nnresolver_, service_name,
						      "_fawkes._tcp",
						      fawkes_port_);
  avahi_thread_->publish_service(fawkes_service);
  delete fawkes_service;
#else
  service_publisher_ = new DummyServicePublisher();
  service_browser_   = new DummyServiceBrowser();
  nnresolver_        = new NetworkNameResolver();
#endif
}


/** Destructor. */
FawkesNetworkManager::~FawkesNetworkManager()
{
  thread_collector_->remove(fawkes_network_thread_);
  delete fawkes_network_thread_;
#ifdef HAVE_AVAHI
  thread_collector_->remove(avahi_thread_);
  delete avahi_thread_;
#else
  delete service_publisher_;
  delete service_browser_;
#endif
  delete nnresolver_;
}


/** Get Fawkes network hub.
 * @return Fawkes network hub
 */
FawkesNetworkHub *
FawkesNetworkManager::hub()
{
  return fawkes_network_thread_;
}


/** Get network name resolver.
 * @return network name resolver
 */
NetworkNameResolver *
FawkesNetworkManager::nnresolver()
{
  return nnresolver_;
}


/** Get service publisher
 * @return service publisher
 */
ServicePublisher *
FawkesNetworkManager::service_publisher()
{
  return service_publisher_;
}


/** Get service browser.
 * @return service browser
 */
ServiceBrowser *
FawkesNetworkManager::service_browser()
{
  return service_browser_;
}

/** Get Fawkes TCP port.
 * @return TCP port on which Fawkes is listening
 */
unsigned short int
FawkesNetworkManager::fawkes_port() const
{
  return fawkes_port_;
}


} // end namespace fawkes
