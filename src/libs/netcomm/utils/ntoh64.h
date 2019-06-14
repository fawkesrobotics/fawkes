
/***************************************************************************
 *  ntoh64.h - Network <-> Host Byte Order conversion for uint64_t
 *
 *  Created: Sat Mar 21 12:48:22 2009
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
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

#ifndef _NETCOMM_UTILS_NTOH64_H_
#define _NETCOMM_UTILS_NTOH64_H_

#include <arpa/inet.h>

#include <stdint.h>

#define ntoh64_(x)                                                  \
	(((uint64_t)(ntohl((uint32_t)(((uint64_t)x << 32) >> 32))) << 32) \
	 | (uint32_t)ntohl(((uint32_t)((uint64_t)x >> 32))))

#define hton64_(x) ntohll(x)

#ifdef __OPTIMIZE__
#	define ntoh64(x) ntoh64_(x)
#	define hton64(x) ntoh64_(x)
#else

inline uint64_t
ntoh64(uint64_t x)
{
	return ntoh64_(x);
}

inline uint64_t
hton64(uint64_t x)
{
	return ntoh64_(x);
}

#endif // __OPTIMIZE__

#endif // NETCOMM_UTILS_NTOH64__
