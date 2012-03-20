
/***************************************************************************
 *  strndup.h - strndup alternative for systems that don't have it
 *
 *  Created: Mon Mar 07 23:39:52 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_MISC_STRNDUP_H_
#define __UTILS_MISC_STRNDUP_H_

#include <cstring>
#include <cstdlib>

#if ! (defined(__USE_XOPEN2K8) || defined(__USE_GNU) || defined(__POSIX_VISIBLE) && __POSIX_VISIBLE >= 200809)
#define __COMPAT_STRNDUP

char * strndup(const char *s, size_t n);

#endif

#endif
