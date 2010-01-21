
/***************************************************************************
 *  bblog.cpp - BBLogger console tool
 *
 *  Created: Thu Jan 21 01:33:45 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
 *
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

#include <utils/system/argparser.h>
#include <utils/system/signal.h>

#include <cstdlib>
#include <cstdio>

using namespace fawkes;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] <watch|meta|print|replay> <logfile>\n"
         " -h  Print this usage information\n"
         "COMMANDS:\n"
         " watch     Continuously watch a log file (like tail)\n"
         " meta      Print meta information of log file\n"
         " print     Print specific data index\n"
         " replay    Replay log file in real-time to console\n",
	 program_name);
}

/** BBLogger tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "h");

  if ( argp.has_arg("h") ) {
    print_usage(argv[0]);
    exit(0);
  }

  return 0;
}
