
/***************************************************************************
 *  qa_filetype.cpp - angle QA app
 *
 *  Created: Sun Oct 26 10:44:55
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

#include <utils/system/filetype.h>
#include <cstdio>

using namespace fawkes;

int
main(int argc, char **argv)
{
  printf("File type: %s\n", filetype_file(argv[0]).c_str());
  printf("Mime type: %s\n", mimetype_file(argv[0]).c_str());

  return 0;
}

