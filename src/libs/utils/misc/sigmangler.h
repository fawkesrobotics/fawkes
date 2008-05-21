
/***************************************************************************
 *  sigmangler.h - C++ Signature Mangler
 *
 *  Created: Sun Apr 20 11:01:55 2008 (GO2008, day 2)
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FVUTILS_MISC_SIGMANGLER_H_
#define __FVUTILS_MISC_SIGMANGLER_H_

namespace fawkes {


class CppSignatureMangler
{
 public:
  static char * strip_class_type(const char *type);
};


} // end namespace fawkes

#endif
