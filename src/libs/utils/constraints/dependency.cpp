
/***************************************************************************
 *  dependency.cpp - dependency constraint
 *
 *  Created: Tue May 29 14:17:52 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <utils/constraints/dependency.h>

/** @class DependencyViolationException <utils/constraints/dependency.h>
 * Dependency violation exception.
 * Thrown if an operation is tried which would violate the dependency
 * constraint.
 */

/** Constructor.
 * @param msg message
 */
DependencyViolationException::DependencyViolationException(char *msg)
  : Exception(msg)
{
}
