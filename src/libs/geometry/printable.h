
/***************************************************************************
 *  printable.h - Printable interface
 *
 *  Created: Tue Oct 07 14:37:22 2008
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

#ifndef __GEOMETRY_PRINTABLE_H_
#define __GEOMETRY_PRINTABLE_H_

#include <ostream>

namespace fawkes {
  class Printable;
}

std::ostream& operator<<(std::ostream& stream, const fawkes::Printable& p);

namespace fawkes {

class Printable
{
  friend std::ostream& ::operator<<(std::ostream& stream, const Printable& p);

 public:
  Printable();
  virtual ~Printable();

 protected:
  virtual std::ostream& print(std::ostream& stream) const =0;
};

}

#endif /* __GEOMETRY_PRINTABLE_H_ */
