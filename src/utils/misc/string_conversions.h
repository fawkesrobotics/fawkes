 
/***************************************************************************
 *  string_conversions.h - string conversions
 *
 *  Created: Thu Oct 12 12:03:49 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_MISC_STRINGTOOLS_H_
#define __UTILS_MISC_STRINGTOOLS_H_

#include <string>

class StringConversions
{
 public:
  static std::string toUpper(std::string str);
  static std::string toLower(std::string str);

  static std::string toString(unsigned int i);
  static std::string toString(int i);
  static std::string toString(float f);
  static std::string toString(double d);
  static std::string toString(bool b);

  static unsigned int toUInt(std::string s);
  static int          toInt(std::string s);
  static float        toFloat(std::string s);
  static double       toDouble(std::string s);
  static bool         toBool(std::string s);

 private:
  // may not be instantiated!
  StringConversions() {};
};



#endif
