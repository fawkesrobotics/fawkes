
/***************************************************************************
 *  addr_comp.h - address comparison
 *
 *  Created: Tue Dec 13 15:17:43 2016
 *  Copyright  2006-2016  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_UTILS_ADDR_COMP_H_
#define __NETCOMM_UTILS_ADDR_COMP_H_

#include <netinet/in.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Compare two sockaddr structures.
 * The comparison is based on address family first, and if the same based on
 * the IP address. Works for AF_INET and AF_INET6.
 * @param a first compare argument
 * @param b second compare argument
 * @return <0 if a is less than b, 0 if they are the same, or >0 otherwise
 */
static inline int
sock_addr_cmp_addr(const struct sockaddr *a, const struct sockaddr *b)
{
	if (a->sa_family != b->sa_family) {
		return a->sa_family - b->sa_family;
	} else if (a->sa_family == AF_INET) {
		return (((sockaddr_in *)a)->sin_addr.s_addr - ((sockaddr_in *)b)->sin_addr.s_addr);
	} else if (a->sa_family == AF_INET6) {
		return (memcmp((char *) &((sockaddr_in6 *)a)->sin6_addr,
		               (char *) &((sockaddr_in6 *)a)->sin6_addr,
		               sizeof(in6_addr)));
	}
}

/** Compare concept comparator for sockaddr.
 * @author Tim Niemueller
 */
struct SockAddrCompare
{
	/** Compare sockaddr structures.
	 * uses sock_addr_cmp_addr().
	 * @param a first compare argument
	 * @param b second compare argument
	 * @return true if a < b, false otherwise
	 */
	bool operator()(const struct sockaddr * & a, const struct sockaddr * & b) const
	{
		return (sock_addr_cmp_addr(a, b) < 0);
	}
};
	
} // end namespace fawkes

#endif
