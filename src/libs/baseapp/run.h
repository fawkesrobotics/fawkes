
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

#include <baseapp/init_options.h>

namespace fawkes {
  class ArgumentParser;
  class MultiLogger;
  class NetworkLogger;
  class BlackBoard;
  class Configuration;
  class FawkesMainThread;
  class PluginManager;
  class AspectManager;
  class ThreadManager;
  class FawkesNetworkManager;
  class Clock;
  class ConfigNetworkHandler;
  class PluginNetworkHandler;

  namespace runtime {
#if 0 /* just to make Emacs auto-indent happy */
  }
}
#endif

extern ArgumentParser       *argument_parser;
extern FawkesMainThread     *main_thread;
extern MultiLogger          *logger;
extern NetworkLogger        *network_logger;
extern BlackBoard           *blackboard;
extern Configuration        *config;
extern Clock                *clock;
extern PluginManager        *plugin_manager;
extern AspectManager        *aspect_manager;
extern ThreadManager        *thread_manager;
extern FawkesNetworkManager *network_manager;
extern ConfigNetworkHandler *nethandler_config;
extern PluginNetworkHandler *nethandler_plugin;

int  init(int argc, char **argv);
int  init(InitOptions options);
void run();
void cleanup();
void quit();

void print_usage(const char *progname);

} // end namespace runtime
} // end namespace fawkes


#endif
