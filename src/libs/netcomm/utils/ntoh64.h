
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

#ifndef __NETCOMM_UTILS_NTOH64_H_
#define __NETCOMM_UTILS_NTOH64_H_

#include <stdint.h>
#include <arpa/inet.h>

#define __ntoh64(x) (((uint64_t)(ntohl((uint32_t)(((uint64_t)x << 32) >> 32))) << 32) | \
		       (uint32_t)ntohl(((uint32_t)((uint64_t)x >> 32))))

#define __hton64(x) ntohll(x)

#ifdef __OPTIMIZE__
#  define ntoh64(x) __ntoh64(x)
#  define hton64(x) __ntoh64(x)
#else

inline uint64_t
ntoh64(uint64_t x)
{
  return __ntoh64(x);
}

inline uint64_t
hton64(uint64_t x)
{
  return __ntoh64(x);
}

#endif // __OPTIMIZE__

#endif // __NETCOMM_UTILS_NTOH64_
