 
/***************************************************************************
 *  string_conversions.cpp - string conversions
 *
 *  Created: Thu Oct 12 12:05:42 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
#include <core/exceptions/system.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <cstdio>
#include <cstdlib>

namespace fawkes {

/** @class StringConversions <utils/misc/string_conversions.h>
 * Utility class that holds string methods.
 * @author Tim Niemueller
 *
 * @fn static std::string StringConversions::to_string(std::string &s)
 * No-op conversion of string.
 * @param s value to convert
 * @return string the very same string
 */

/** Convert string to all-uppercase string.
 * @param str string to convert
 * @return converted string
 */
std::string
StringConversions::to_upper(std::string str)
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
StringConversions::to_lower(std::string str)
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
StringConversions::to_string(const unsigned int i)
{
  char *tmp;
  std::string rv;
  if (asprintf(&tmp, "%u", i) == -1) {
    throw OutOfMemoryException("StringConversions::tostring(const unsigned int): asprintf() failed");
  }
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert int value to a string.
 * @param i value to convert
 * @return string representation of value.
 */
std::string
StringConversions::to_string(const int i)
{
  char *tmp;
  std::string rv;
  if (asprintf(&tmp, "%i", i) == -1) {
    throw OutOfMemoryException("StringConversions::tostring(const int): asprintf() failed");
  }
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert long int value to a string.
 * @param i value to convert
 * @return string representation of value.
 */
std::string
StringConversions::to_string(const long int i)
{
  char *tmp;
  std::string rv;
  if (asprintf(&tmp, "%li", i) == -1) {
    throw OutOfMemoryException("StringConversions::tostring(const long int): asprintf() failed");
  }
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert float value to a string.
 * @param f value to convert
 * @return string representation of value.
 */
std::string
StringConversions::to_string(const float f)
{
  char *tmp;
  std::string rv;
  if (asprintf(&tmp, "%f", f) == -1) {
    throw OutOfMemoryException("StringConversions::tostring(const float): asprintf() failed");
  }
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert double value to a string.
 * @param d value to convert
 * @return string representation of value.
 */
std::string
StringConversions::to_string(const double d)
{
  char *tmp;
  std::string rv;
  if (asprintf(&tmp, "%f", d) == -1) {
    throw OutOfMemoryException("StringConversions::tostring(const double d): asprintf() failed");
  }
  rv = tmp;
  free(tmp);
  return rv;
}


/** Convert bool value to a string.
 * @param b value to convert
 * @return string representation of value.
 */
std::string
StringConversions::to_string(const bool b)
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
StringConversions::to_uint(std::string s)
{
  unsigned int l = atoll(s.c_str());
  return l;
}


/** Convert string to an int value
 * @param s string to convert
 * @return value as represented by string
 */
int
StringConversions::to_int(std::string s)
{
  return atoi(s.c_str());
}


/** Convert string to a float value
 * @param s string to convert
 * @return value as represented by string
 */
float
StringConversions::to_float(std::string s)
{
  return (float)atof(s.c_str());
}


/** Convert string to a double value
 * @param s string to convert
 * @return value as represented by string
 */
double
StringConversions::to_double(std::string s)
{
  return atof(s.c_str());
}


/** Convert string to a bool value
 * @param s string to convert
 * @return value as represented by string
 */
bool
StringConversions::to_bool(std::string s)
{
  if ( (s == "true") ||
       (s == "yes") ||
       (s == "1") ) {
    return true;
  } else {
    return false;
  }
}

/** Trim string.
 * Removes spaces at beginning and end of string.
 * @param s string to trim, upon return contains trimmed string
 */
void
StringConversions::trim_inplace(std::string &s)
{
  std::string::size_type p1 = s.find_first_not_of(' ');
  std::string::size_type p2 = s.find_last_not_of(' ');
  s = s.substr(p1 == std::string::npos ? 0 : p1, 
	       p2 == std::string::npos ? s.length() - 1 : p2 - p1 + 1);
}


/** Trim spring.
 * Removes spaces at beginning and end of string.
 * @param s string to trim
 * @return trimmed string
 */
std::string
StringConversions::trim(std::string &s)
{
  std::string::size_type p1 = s.find_first_not_of(' ');
  std::string::size_type p2 = s.find_last_not_of(' ');
  return s.substr(p1 == std::string::npos ? 0 : p1, 
		  p2 == std::string::npos ? s.length() - 1 : p2 - p1 + 1);
}


} // end namespace fawkes
