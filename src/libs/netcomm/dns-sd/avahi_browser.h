
/***************************************************************************
 *  avahi_browser.h - browse services via avahi
 *
 *  Created: Wed Nov 08 13:05:34 2006
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

#ifndef __NETCOMM_DNSSD_AVAHI_BROWSER_H_
#define __NETCOMM_DNSSD_AVAHI_BROWSER_H_

#include <netcomm/dns-sd/avahi_service.h>
#include <netcomm/dns-sd/avahi_browse_handler.h>

#include <avahi-client/client.h>
#include <avahi-client/lookup.h>

#include <map>
#include <list>
#include <string>
#include <utility>

class Mutex;

class AvahiBrowser
{
 friend class AvahiThread;

 public:
  AvahiBrowser();
  ~AvahiBrowser();

  void add_handler(const char *service_type, AvahiBrowseHandler *h);
  void remove_handler(const char *service_type, AvahiBrowseHandler *h);

 private:
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


  std::map<std::string, std::list<AvahiBrowseHandler *> > handlers;
  std::map<std::string, AvahiServiceBrowser * > browsers;

  AvahiClient      *client;
  Mutex            *mutex;
};


#endif
