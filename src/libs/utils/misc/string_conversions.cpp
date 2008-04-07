 
/***************************************************************************
 *  string_conversions.cpp - string conversions
 *
 *  Created: Thu Oct 12 12:05:42 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <utils/misc/string_conversions.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <cstdio>
#include <cstdlib>


/** @class StringConversions utils/misc/string_conversions.h
 * Utility class that holds string methods.
 */

/** Convert string to all-uppercase string.
 * @param str string to convert
 * @return converted string
 */
std::string
StringConversions::toUpper(std::string str)
{
  for(unsigned int i = 0; i < str.length(); ++i) {
    str[i] = (char)toupper(str[i]);
  }
  return str;
}

 
/** Convert string to all-lowercase string.
 * @param str string to convert
 * @return converted string
 */
std::string
StringConversions::toLower(std::string str)
{
   for(unsigned int i = 0; i < str.length(); ++i) {
     str[i] = (char)tolower(str[i]);
   }
   return str;
}


/** Convert unsigned int value to a string.
 * @param i value to convert
 * @return string representation of value.
 */
std::string
StringConversions::toString(const unsigned int i)
{
  char *tmp;
  std::string rv;
  asprintf(&tmp, "%u", i);
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert int value to a string.
 * @param i value to convert
 * @return string representation of value.
 */
std::string
StringConversions::toString(const int i)
{
  char *tmp;
  std::string rv;
  asprintf(&tmp, "%i", i);
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert float value to a string.
 * @param f value to convert
 * @return string representation of value.
 */
std::string
StringConversions::toString(const float f)
{
  char *tmp;
  std::string rv;
  asprintf(&tmp, "%f", f);
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert double value to a string.
 * @param d value to convert
 * @return string representation of value.
 */
std::string
StringConversions::toString(const double d)
{
  char *tmp;
  std::string rv;
  asprintf(&tmp, "%f", d);
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert bool value to a string.
 * @param b value to convert
 * @return string representation of value.
 */
std::string
StringConversions::toString(const bool b)
{
  if ( b ) {
    return std::string("true");
  } else {
    return std::string("false");
  }
}


/** Convert string to an unsigned int value
 * @param s string to convert
 * @return value as represented by string
 */
unsigned int
StringConversions::toUInt(std::string s)
{
  unsigned int l = atoll(s.c_str());
  return l;
}


/** Convert string to an int value
 * @param s string to convert
 * @return value as represented by string
 */
int
StringConversions::toInt(std::string s)
{
  return atoi(s.c_str());
}


/** Convert string to a float value
 * @param s string to convert
 * @return value as represented by string
 */
float
StringConversions::toFloat(std::string s)
{
  return (float)atof(s.c_str());
}


/** Convert string to a double value
 * @param s string to convert
 * @return value as represented by string
 */
double
StringConversions::toDouble(std::string s)
{
  return atof(s.c_str());
}


/** Convert string to a bool value
 * @param s string to convert
 * @return value as represented by string
 */
bool
StringConversions::toBool(std::string s)
{
  if ( (s == "true") ||
       (s == "yes") ||
       (s == "1") ) {
    return true;
  } else {
    return false;
  }
}
