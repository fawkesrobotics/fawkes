
/***************************************************************************
 *  printable.cpp - Printable interface
 *
 *  Created: Thu Oct 09 10:43:59 2008
 *  Copyright  2008  Daniel Beck
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

#include <geometry/printable.h>

namespace fawkes {

/** @class fawkes::Printable <geometry/printable.h>
 * Interface class for printable objects. Printable objects can be
 * printed by means of the <<-operator.
 * 
 * @author Daniel Beck
 */

/** @fn std::ostream& fawkes::Printable::print(std::ostream& stream) const
 * This method is called by the overloaded <<-operator.
 * @param stream the output stream
 * @return reference to the output stream
 */

/** Constructor. */
Printable::Printable()
{
}

/** Destructor. */
Printable::~Printable()
{
}

} // end namespace fawkes

/** Overloaded <<-operator that calls the print() method of the given
 * Printable object.
 * @param stream the output stream
 * @param p the Printable object
 * @return the output stream
 */
std::ostream& operator<<(std::ostream& stream, const fawkes::Printable& p)
{
  return p.print(stream);
}

