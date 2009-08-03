
/***************************************************************************
 *  pathparser.cpp - Header for path parser
 *
 *  Created: Mon Jul 07 13:25:10 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <utils/system/pathparser.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>

using namespace std;

namespace fawkes {

/** @class PathParser <utils/system/pathparser.h>
 * Path parser.
 * Parses a given (Unix) file system path and provides the elements and vector
 * elements.
 * @author Tim Niemueller
 */

/** Constructor (C++ string).
 * @param path path to parse
 */
PathParser::PathParser(std::string &path)
{
  ctor(path);
}


/** Constructor (C string).
 * @param path path to parse
 */
PathParser::PathParser(const char *path)
{
  std::string spath = path;
  ctor(spath);
}


void
PathParser::ctor(const std::string &path)
{
  __abs_path  = false;

  char *p = strdup(path.c_str());
  char *saveptr;
  char *r = strtok_r(p, "/", &saveptr);

  if ( ! r ) {
    // single string, no slash, does not end with slash
    push_back(p);
  } else {
    __abs_path = ( r != p );

    while ( r ) {
      if ( strlen(r) > 0 ) {
	push_back(r);
      }
      r = strtok_r(NULL, "/", &saveptr);
    }
  }

  free(p);
}


/** Debug print to stdout. */
void
PathParser::print_debug()
{
  for (size_type i = 0; i < size(); ++i) {
    printf("Path element: %s\n", ((*this)[i]).c_str());
  }
}

/** Get path as string.
 * Joins the path elements to one path again.
 * @return path as string
 */
std::string
PathParser::path_as_string()
{
  string rv = __abs_path ? "/" : "";

  size_type sz = size();

  if ( sz > 0 ) {
    rv += (*this)[0];
  }

  for (size_type i = 1; i < sz; ++i) {
    rv += "/" + (*this)[i];
  }

  return rv;
}


/** Check if path is absolute.
 * @return true if path is absolute, false otherwise
 */
bool
PathParser::is_absolute() const
{
  return __abs_path;
}

} // end namespace fawkes
