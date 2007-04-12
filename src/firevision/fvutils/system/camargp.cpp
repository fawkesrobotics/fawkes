
/***************************************************************************
 *  camargp.cpp - Camera argument parser
 *
 *  Created: Wed Apr 11 15:47:34 2007
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <fvutils/system/camargp.h>

#include <iostream>

using namespace std;

/** @class CameraArgumentParser <fvutils/system/camargp.h>
 * Camera argument parser.
 * Simple parser that will parse a camera parameter string that defines
 * the camera type and options specific to this camera.
 *
 * In general a string is of the form
 * @code
 * camera:param1=value1:param2=value2:arg1:arg2
 * @endcode
 * The string is a colon-separated (:) list of elements.
 *
 * The first element (camera in the example) denotes the camera type.
 * See the CameraFactory documentation for allowed values. It can be queried
 * with the camid() method.
 *
 * The rest is a list of parameters and arguments. Parameters are key/value
 * pairs separated by an equals sign. The are then queried with the has(),
 * get() and parameters() methods. Arguments are simple strings that do not contain
 * an equals sign and are given as-is via the arguments() method. These could
 * for example be a list of files etc..
 *
 * @see CameraFactory
 * @author Tim Niemueller
 */

/** Constructor.
 * @param as camera argument string
 */
CameraArgumentParser::CameraArgumentParser(const char *as)
{
  values.clear();

  std::string s = as;
  s += ":";

  _camid = s;
  string::size_type start;
  string::size_type end;
  if ( (end = s.find(":", 0)) != string::npos ) {
    _camid = s.substr(0, end);
  }
  start = end + 1;

  while ( (end = s.find(":", start)) != string::npos ) {
    string t = s.substr(start, (end - start));
    string::size_type e;
    if ( (e = t.find("=", 0)) != string::npos ) {
      if ( (e > 0 ) && (e < t.length() - 1) ) {
	values[t.substr(0, e)] = t.substr(e+1);
      }
    } else {
      if ( t != "" ) {
	args.push_back(t);
      }
    }
    start = end + 1;
  }
}


/** Destructor. */
CameraArgumentParser::~CameraArgumentParser()
{
  values.clear();
  args.clear();
}


/** Get camera ID.
 * Get the camera ID. This is the very first element before
 * the first colon.
 * @return camera ID string
 */
std::string
CameraArgumentParser::camid() const
{
  return _camid;
}


/** Check if an parameter was given.
 * Checks if the given parameter s was given in the argument
 * string.
 * @param s parameter key to check for
 * @return true, if the parameter has been supplied, false otherwise
 */
bool
CameraArgumentParser::has(std::string s) const
{
  return (values.find(s) != values.end());
}


/** Get the value of the given parameter.
 * @param s key of the parameter to retrieve
 * @return the value of the given parameter or an empty string if the
 * parameter was not supplied.
 */
std::string
CameraArgumentParser::get(std::string s)
{
  if ( values.find(s) != values.end() ) {
    return values[s];
  } else {
    return string();
  }
}


/** Get the arguments.
 * Returns a vector of arguments supplied in the argument string.
 * @return vector of arguments
 */
std::vector<std::string>
CameraArgumentParser::arguments() const
{
  return args;
}


/** Get a map of parameters.
 * @returns map of key/value pairs of parameters supplied in the argument string.
 */
std::map<std::string, std::string>
CameraArgumentParser::parameters() const
{
  return values;
}
