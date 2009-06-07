
/***************************************************************************
 *  network_manager.h - Fawkes network manager
 *
 *  Created: Wed Nov 15 23:52:40 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FAWKES_NETWORK_MANAGER_H_
#define __FAWKES_NETWORK_MANAGER_H_

class FawkesThreadManager;
namespace fawkes {
  class FawkesNetworkServerThread;
  class FawkesNetworkHandler;
  class FawkesNetworkHub;
  class AvahiThread;
  class NetworkNameResolver;
  class ServicePublisher;
  class ServiceBrowser;
}

class FawkesNetworkManager
{
 public:
  FawkesNetworkManager(FawkesThreadManager *thread_manager,
		       unsigned short int fawkes_port,
		       const char *service_name);
  ~FawkesNetworkManager();

  fawkes::FawkesNetworkHub *     hub();
  fawkes::NetworkNameResolver *  nnresolver();
  fawkes::ServicePublisher *     service_publisher();
  fawkes::ServiceBrowser *       service_browser();

 private:
  unsigned short int                  fawkes_port;
  FawkesThreadManager                *thread_manager;
  fawkes::FawkesNetworkServerThread  *fawkes_network_thread;
  fawkes::AvahiThread                *avahi_thread;

  fawkes::NetworkNameResolver        *_nnresolver;
  fawkes::ServicePublisher           *_service_publisher;
  fawkes::ServiceBrowser             *_service_browser;
};

#endif
