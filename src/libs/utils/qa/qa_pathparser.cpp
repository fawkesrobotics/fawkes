
/***************************************************************************
 *  qa_pathparser.cpp - QA for PathParser
 *
 *  Created: Mon Jul 07 14:04:46 2008
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
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

// Do not include in api reference
///@cond QA

#include <utils/system/pathparser.h>
#include <cstdio>

using namespace fawkes;

int
main(int argc, char **argv)
{

  printf("Trying /this/is/a/test/path foo/bar\n");
  PathParser pp1("/this/is/a/test/path foo/bar");
  pp1.print_debug();
  printf("Re-generated: %s\n", pp1.path_as_string().c_str());

  printf("Trying relative/path/test\n");
  PathParser pp2("relative/path/test");
  pp2.print_debug();
  printf("Re-generated: %s\n", pp2.path_as_string().c_str());

  printf("Trying relative/path/test/with/end/slash/\n");
  PathParser pp3("relative/path/test/with/end/slash/");
  pp3.print_debug();
  printf("Re-generated: %s\n", pp3.path_as_string().c_str());

  return 0;
}

/// @endcond
