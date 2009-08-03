
/***************************************************************************
 *  avahi_resolver_handler.h - Avahi name resolver
 *
 *  Created: Tue Nov 14 14:32:56 2006 (as avahi_resolver.h)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_DNSSD_AVAHI_RESOLVER_HANDLER_H_
#define __NETCOMM_DNSSD_AVAHI_RESOLVER_HANDLER_H_

#include <sys/socket.h>

namespace fawkes {

class AvahiResolverHandler
{
 public:
  virtual ~AvahiResolverHandler();
  virtual void resolved_name(char *name,
			     struct sockaddr *addr, socklen_t addrlen)   = 0;
  virtual void resolved_address(struct sockaddr_in *addr, socklen_t addrlen,
				char *name)                              = 0;

  virtual void name_resolution_failed(char *name)                        = 0;
  virtual void address_resolution_failed(struct sockaddr_in *addr,
					 socklen_t addrlen)              = 0;
};

} // end namespace fawkes

#endif
