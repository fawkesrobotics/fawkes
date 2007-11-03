
/***************************************************************************
 *  resolver.h - Fawkes network name resolver
 *
 *  Created: Tue Nov 14 14:25:52 2006
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

#ifndef __NETCOMM_UTILS_RESOLVER_H_
#define __NETCOMM_UTILS_RESOLVER_H_

#include <core/utils/lock_hashmap.h>
#include <core/utils/lock_map.h>
#include <utils/misc/string_compare.h>

#include <sys/socket.h>
#include <cstddef>

#include <ctime>
#include <string>
#include <utility>

class AvahiThread;
class NetworkNameResolverThread;

class NetworkNameResolver
{
 friend class NetworkNameResolverThread;

 public:
  NetworkNameResolver(AvahiThread *avahi_thread = NULL);
  ~NetworkNameResolver();

  bool resolve_name(const char *name, struct sockaddr **addr, socklen_t *addrlen);
  bool resolve_name_blocking(const char *name, struct sockaddr **addr, socklen_t *addrlen);
  bool resolve_address(struct sockaddr *addr, socklen_t addr_len, char **name);

  void flush_cache();

 private:
  void name_resolved(char *name, struct sockaddr *addr, socklen_t addrlen);
  void addr_resolved(struct sockaddr *addr, socklen_t addrlen, char *name, bool namefound);
  void name_resolution_failed(char *name);
  void address_resolution_failed(struct sockaddr *addr, socklen_t addrlen);

 private:
  NetworkNameResolverThread *resolver_thread;

  LockHashMap<uint32_t, std::pair<char *, time_t> >       addr2name_cache;
  LockHashMap<char *,
             std::pair<struct sockaddr *, time_t>,
             __gnu_cxx::hash<char *>, StringEquality >    name2addr_cache;

  LockHashMap<uint32_t, std::pair<char *, time_t> >::iterator  a2ncit;
  LockHashMap<char *, std::pair<struct sockaddr *, time_t>,
    __gnu_cxx::hash<char *>, StringEquality >::iterator        n2acit;
};

#endif
