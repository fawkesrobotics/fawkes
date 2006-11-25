
/***************************************************************************
 *  avahi_thread.h - Avahi Thread
 *
 *  Created: Wed Nov 08 11:17:06 2006
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

#ifndef __NETCOMM_DNSSD_AVAHI_THREAD_H_
#define __NETCOMM_DNSSD_AVAHI_THREAD_H_

#include <netcomm/dns-sd/avahi_service.h>
#include <core/threading/thread.h>

#include <avahi-client/client.h>

class AvahiServicePublisher;
class AvahiBrowser;
class AvahiBrowseHandler;

class AvahiThread : public Thread
{
 public:
  AvahiThread();
  ~AvahiThread();

  void publish(AvahiService *service);
  void watch(const char *service_type, AvahiBrowseHandler *h);
  void unwatch(const char *service_type, AvahiBrowseHandler *h);

  virtual void loop();

 private:
  typedef struct AvahiSimplePoll AvahiSimplePoll;

  static void client_callback(AvahiClient *c, AvahiClientState state, void *instance);
  void recover();

  AvahiSimplePoll  *simple_poll;
  AvahiClient      *client;

  AvahiServicePublisher *service_publisher;
  AvahiBrowser          *browser;
};


#endif
