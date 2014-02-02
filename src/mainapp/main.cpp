
/***************************************************************************
 *  main.cpp - Fawkes main application
 *
 *  Created: Thu Nov  2 16:44:48 2006
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#include <baseapp/run.h>
#include <core/exception.h>
#include <cstdio>


/** Fawkes application.
 * @param argc argument count
 * @param argv array of arguments
 */
int
main(int argc, char **argv)
{
  try {
    int retval = 0;
    if (! fawkes::runtime::init(argc, argv, retval)) {
      return retval;
    }
    fawkes::runtime::run();
    fawkes::runtime::cleanup();
  } catch (fawkes::Exception &e) {
    printf("Fawkes execution ended unexpectedly, exception follows.\n");
    e.print_trace();
    return 1;
  }

  return 0;
}
