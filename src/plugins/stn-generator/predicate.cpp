
/***************************************************************************
 *  predicate.cpp - stn-generator
 *
 *  Created: Sat May  6 20:16:21 2017
 *  Copyright  2017  Matthias Loebach
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "predicate.h"

namespace fawkes {
namespace stn {

/** @class Predicate "predicate.h"
 * A representation of a Predicate in the STN.
 */

/** Constructor.
 * @param name The name of the predicate.
 * @param condition False iff this predicate is negated.
 * @param attrs Parameters of the predicate.
 */
Predicate::Predicate(std::string name, bool condition, std::vector<std::string> attrs)
{
  name_ = name;
  condition_ = condition;
  attrs_ = attrs;
}

/** Print a Predicate.
 * This prints all relevant information about the predicate.
 * @param strm The stream to pass the information to.
 * @param a The predicate to show.
 */
std::ostream&
operator<<(std::ostream &strm, const Predicate &a)
{
  strm << "\t" << a.name_ << "," << a.condition_;
  for ( std::string s : a.attrs_ ) {
    strm << "," << s;
  }
  strm << std::endl;

  return strm;
}

/** Compare two Predicates.
 * @param rhs The other predicatge.
 * @return True iff the two predicates have the same properties.
 */
bool
Predicate::operator==(const Predicate& rhs)
{
  return ( ( name_ == rhs.name_ ) && ( condition_ == rhs.condition_ ) && ( attrs_ == rhs.attrs_ ) );
}

/** Get the name of the predicate.
 * @return The name of the predicate.
 */
std::string
Predicate::name()
{
  return name_;
}

/** Get the condition of the predicate.
 * @return True iff the predicate's condition is true.
 */
bool
Predicate::condition()
{
  return condition_;
}

/** Get the attributes of the predicate.
 * @return A vector of attributes as strings.
 */
std::vector<std::string>
Predicate::attrs()
{
  return attrs_;
}
}
}
