
/***************************************************************************
 *  avahi_resolver.cpp - Avahi name resolver
 *
 *  Created: Tue Nov 14 14:32:56 2006
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

/// @cond UNFINISHED

#ifndef __NETCOMM_DNSSD_AVAHI_RESOLVER_H_
#define __NETCOMM_DNSSD_AVAHI_RESOLVER_H_

#include <avahi-client/client.h>
#include <avahi-client/lookup.h>

class AvahiNameResolver
{
  friend class AvahiThread;

 public:
  AvahiNameResolver();
  ~AvahiNameResolver();

  bool resolve(const char *name, AvahiAddress *addr);

 private:
  AvahiClient *client;
};

#endif

/// @endcond
