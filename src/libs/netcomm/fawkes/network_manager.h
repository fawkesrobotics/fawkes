
/***************************************************************************
 *  network_manager.h - Fawkes network manager
 *
 *  Created: Wed Nov 15 23:52:40 2006
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

#ifndef _FAWKES_NETWORK_MANAGER_H_
#define _FAWKES_NETWORK_MANAGER_H_

#include <string>

namespace fawkes {
class ThreadCollector;
class FawkesNetworkServerThread;
class FawkesNetworkHandler;
class FawkesNetworkHub;
class AvahiThread;
class NetworkNameResolver;
class ServicePublisher;
class ServiceBrowser;

class FawkesNetworkManager
{
public:
	FawkesNetworkManager(ThreadCollector   *thread_collector,
	                     bool               enable_ipv4,
	                     bool               enable_ipv6,
	                     const std::string &listen_ipv4,
	                     const std::string &listen_ipv6,
	                     unsigned short int fawkes_port,
	                     const char        *service_name);
	~FawkesNetworkManager();

	FawkesNetworkHub    *hub();
	NetworkNameResolver *nnresolver();
	ServicePublisher    *service_publisher();
	ServiceBrowser      *service_browser();

	unsigned short int fawkes_port() const;

private:
	unsigned short int         fawkes_port_;
	ThreadCollector           *thread_collector_;
	FawkesNetworkServerThread *fawkes_network_thread_;
	AvahiThread               *avahi_thread_;

	NetworkNameResolver *nnresolver_;
	ServicePublisher    *service_publisher_;
	ServiceBrowser      *service_browser_;
};

} // end namespace fawkes

#endif
