
/***************************************************************************
 *  main.cpp - Fawkes plugin tool main
 *
 *  Created: Tue Nov 22 00:25:26 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "plugin_tool.h"
#include <netcomm/fawkes/client.h>

#include <core/threading/thread.h>
#include <utils/system/argparser.h>
#include <utils/system/signal.h>

#include <cstdlib>

int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "hl:u:r:waL");

  if ( argp.has_arg("h") ) {
    PluginTool::print_usage(argp.program_name());
    exit(0);
  }

  Thread::init_main();

  FawkesNetworkClient *c = new FawkesNetworkClient("localhost", 1910);
  c->connect();

  PluginTool *pt = new PluginTool(&argp, c);
  SignalManager::register_handler(SIGINT, pt);
  pt->run();
  SignalManager::finalize();
  delete pt;

  c->disconnect();
  delete c;

  Thread::destroy_main();

  return 0;
}
