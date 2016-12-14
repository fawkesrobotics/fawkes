
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

#ifndef __NETCOMM_UTILS_ADDR_SIZE_H_
#define __NETCOMM_UTILS_ADDR_SIZE_H_

#include <netinet/in.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Get canonical size of sockaddr structure.
 * @param a sockaddr_in or sockaddr_in6 structure with properly set
 * address family field.
 * @return size in bytes of struct
 */
inline size_t
sock_addr_size(const struct sockaddr *a)
{
	if (a->sa_family == AF_INET) {
		return sizeof(sockaddr_in);
	} else if (a->sa_family == AF_INET6) {
		return sizeof(sockaddr_in6);
	} else {
		return 0;
	}
}
	
} // end namespace fawkes

#endif
