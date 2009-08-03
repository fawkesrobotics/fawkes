
/***************************************************************************
 *  dependency.cpp - dependency constraint
 *
 *  Created: Tue May 29 14:17:52 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <utils/constraints/dependency.h>

namespace fawkes {

/** @class DependencyViolationException <utils/constraints/dependency.h>
 * Dependency violation exception.
 * Thrown if an operation is tried which would violate the dependency
 * constraint.
 */

/** Constructor.
 * @param msg message
 */
DependencyViolationException::DependencyViolationException(const char *msg)
  : Exception(msg)
{
}


} // end namespace fawkes
