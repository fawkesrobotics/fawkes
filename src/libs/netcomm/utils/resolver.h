
/***************************************************************************
 *  resolver.h - Fawkes network name resolver
 *
 *  Created: Tue Nov 14 14:25:52 2006
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_UTILS_RESOLVER_H_
#define __NETCOMM_UTILS_RESOLVER_H_

#include <core/utils/lock_hashmap.h>
#include <core/utils/lock_map.h>
#include <utils/misc/string_compare.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <cstddef>

#include <ctime>
#include <string>
#include <utility>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class AvahiThread;
class NetworkNameResolverThread;
class HostInfo;

class NetworkNameResolver
{
 friend class NetworkNameResolverThread;

 public:
  NetworkNameResolver(AvahiThread *avahi_thread = NULL);
  ~NetworkNameResolver();

  bool resolve_name(const char *name, struct sockaddr **addr, socklen_t *addrlen);
  bool resolve_name_blocking(const char *name, struct sockaddr **addr, socklen_t *addrlen);
  bool resolve_address(struct sockaddr *addr, socklen_t addr_len, std::string &name);

  void flush_cache();
  void set_cache_timeout(unsigned int sec);
  unsigned int cache_timeout();

  const char * hostname();
  const char * short_hostname();

 private:
  void name_resolved(char *name, struct sockaddr *addr, socklen_t addrlen);
  void addr_resolved(struct sockaddr *addr, socklen_t addrlen, char *name, bool namefound);
  void name_resolution_failed(char *name);
  void address_resolution_failed(struct sockaddr *addr, socklen_t addrlen);

 private:
  NetworkNameResolverThread *resolver_thread;
  HostInfo *__host_info;
  unsigned int __cache_timeout;

  LockHashMap<uint32_t, std::pair<char *, time_t> >       addr2name_cache;
  LockHashMap<char *,
    std::pair<struct sockaddr *, time_t>,
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
             std::tr1::hash<char *>,
#else
             __gnu_cxx::hash<char *>,
#endif
             StringEquality >                             name2addr_cache;

  LockHashMap<uint32_t, std::pair<char *, time_t> >::iterator  a2ncit;
  LockHashMap<char *, std::pair<struct sockaddr *, time_t>,
#if __GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ > 2)
    std::tr1::hash<char *>,
#else
    __gnu_cxx::hash<char *>,
#endif
    StringEquality >::iterator                                 n2acit;
};

} // end namespace fawkes

#endif
