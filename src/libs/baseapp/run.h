
/***************************************************************************
 *  run.h - Fawkes run functions
 *
 *  Created: Wed May 04 23:23:23 2011
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

#ifndef __LIBS_BASEAPP_RUN_H_
#define __LIBS_BASEAPP_RUN_H_

namespace fawkes {
  class ArgumentParser;
  class MultiLogger;
  class BlackBoard;
  class SQLiteConfiguration;
  class FawkesMainThread;

  namespace runtime {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

extern ArgumentParser       *argument_parser;
extern FawkesMainThread     *main_thread;
extern MultiLogger          *logger;
extern BlackBoard           *blackboard;
extern SQLiteConfiguration  *config;

int  init(int argc, char **argv);
void run();
void cleanup();

void print_usage(const char **progname);

} // end namespace runtime
} // end namespace fawkes


#endif
