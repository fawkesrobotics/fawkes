
/***************************************************************************
 *  network_manager.cpp - Fawkes network manager
 *
 *  Created: Wed Nov 16 00:05:18 2006
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

#include <mainapp/network_manager.h>

#include <mainapp/thread_manager.h>
#include <netcomm/fawkes/server_thread.h>
#include <netcomm/fawkes/handler.h>
#include <netcomm/utils/resolver.h>
#include <utils/logging/liblogger.h>
#ifdef HAVE_AVAHI
#include <netcomm/dns-sd/avahi_thread.h>
#include <netcomm/service_discovery/service.h>
#else
#include <netcomm/service_discovery/dummy_service_publisher.h>
#include <netcomm/service_discovery/dummy_service_browser.h>
#endif

#include <cstdlib>

/** @class FawkesNetworkManager mainapp/network_manager.h
 * Fawkes Network Manager.
 * This class provides a manager for network connections used in Fawkes.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param thread_manager thread manager that threads shall be registered to
 * @param fawkes_port port to listen on for Fawkes network connections
 */
FawkesNetworkManager::FawkesNetworkManager(FawkesThreadManager *thread_manager,
					   unsigned short int fawkes_port)
{
  this->fawkes_port    = fawkes_port;
  this->thread_manager = thread_manager;
  fawkes_network_thread = new FawkesNetworkServerThread(thread_manager, fawkes_port);
  thread_manager->add(fawkes_network_thread);
#ifdef HAVE_AVAHI
  avahi_thread          = new AvahiThread();
  _service_publisher     = avahi_thread;
  _service_browser       = avahi_thread;
  thread_manager->add(avahi_thread);
  _nnresolver = new NetworkNameResolver(avahi_thread);
  char *fawkes_service_name;
  asprintf(&fawkes_service_name, "Fawkes on %s", _nnresolver->short_hostname());
  NetworkService *fawkes_service = new NetworkService(fawkes_service_name,
						      "_fawkes._tcp", fawkes_port);
  free(fawkes_service_name);
  avahi_thread->publish_service(fawkes_service);
  delete fawkes_service;
#else
  LibLogger::log_warn("FawkesNetworkManager", "Avahi not available, only using dummies "
		      "for service publishing/browsing.");
  _service_publisher = new DummyServicePublisher();
  _service_browser   = new DummyServiceBrowser();
  _nnresolver        = new NetworkNameResolver();
#endif
}


/** Destructor. */
FawkesNetworkManager::~FawkesNetworkManager()
{
  thread_manager->remove(fawkes_network_thread);
  delete fawkes_network_thread;
#ifdef HAVE_AVAHI
  thread_manager->remove(avahi_thread);
  delete avahi_thread;
#else
  delete _service_publisher;
  delete _service_browser;
#endif
  delete _nnresolver;
}


/** Get Fawkes network hub.
 * @return Fawkes network hub
 */
FawkesNetworkHub *
FawkesNetworkManager::hub()
{
  return fawkes_network_thread;
}


/** Get network name resolver.
 * @return network name resolver
 */
NetworkNameResolver *
FawkesNetworkManager::nnresolver()
{
  return _nnresolver;
}


/** Get service publisher
 * @return service publisher
 */
ServicePublisher *
FawkesNetworkManager::service_publisher()
{
  return _service_publisher;
}


/** Get service browser.
 * @return service browser
 */
ServiceBrowser *
FawkesNetworkManager::service_browser()
{
  return _service_browser;
}
