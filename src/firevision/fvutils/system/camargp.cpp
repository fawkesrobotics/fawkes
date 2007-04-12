
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


CameraArgumentParser::~CameraArgumentParser()
{
  values.clear();
}


std::string
CameraArgumentParser::camid() const
{
  return _camid;
}


bool
CameraArgumentParser::has(std::string s) const
{
  return (values.find(s) != values.end());
}


std::string
CameraArgumentParser::get(std::string s)
{
  if ( values.find(s) != values.end() ) {
    return values[s];
  } else {
    return string();
  }
}


std::vector<std::string>
CameraArgumentParser::arguments() const
{
  return args;
}


std::map<std::string, std::string>
CameraArgumentParser::parameters() const
{
  return values;
}
