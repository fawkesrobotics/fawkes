
/***************************************************************************
 *  assert_exception.h - exception using assert
 *
 *  Created: Mon Jun 11 19:07:10 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_ASSERT_EXCEPTION_H_
#define __CORE_ASSERT_EXCEPTION_H_

#include <core/exception.h>

#ifdef _ASSERT_H
#  undef assert
#else
#  if defined __cplusplus && __GNUC__ >= 3
#    define __ASSERT_VOID_CAST static_cast<void>
#  else
#    define __ASSERT_VOID_CAST (void)
#  endif
#endif

#define assert(expr)                                                    \
  ((expr)                                                               \
   ? __ASSERT_VOID_CAST (0)                                             \
   : throw fawkes::Exception("Assertion '%s' failed in %s:%u",          \
                             __STRING(expr), __FILE__, __LINE__))

#endif
