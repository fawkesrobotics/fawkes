
/***************************************************************************
 *  strndup.cpp - strndup alternative for systems that don't have it
 *
 *  Created: Tue Mar 08 00:00:27 2011
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

#include <utils/misc/strndup.h>

#ifdef __COMPAT_STRNDUP
char *
strndup(const char *s, size_t n)
{
  size_t l = strlen(s);

  if (l <= n) return strdup(s);

  char *rv = (char *)malloc(n + 1);
  strncpy(rv, s, n);
  rv[n] = 0;
  return rv;
}
#endif

