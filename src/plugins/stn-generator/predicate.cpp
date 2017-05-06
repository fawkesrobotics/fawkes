
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

namespace stn {

Predicate::Predicate(std::string name, bool condition, std::vector<std::string> attrs)
{
  name_ = name;
  condition_ = condition;
  attrs_ = attrs;
}

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

bool
Predicate::operator==(const Predicate& rhs)
{
  return ( ( name_ == rhs.name_ ) && ( condition_ == rhs.condition_ ) && ( attrs_ == rhs.attrs_ ) );
}

std::string
Predicate::name()
{
  return name_;
}

bool
Predicate::condition()
{
  return condition_;
}

std::vector<std::string>
Predicate::attrs()
{
  return attrs_;
}
}
