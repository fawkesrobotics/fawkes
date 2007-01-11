
/***************************************************************************
 *  main.cpp - Fawkes plugin tool main
 *
 *  Created: Tue Nov 22 00:25:26 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include "plugin_tool.h"
#include <netcomm/fawkes/client.h>

#include <utils/system/argparser.h>
#include <utils/system/signal.h>

int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "l:u:w");

  FawkesNetworkClient *c = new FawkesNetworkClient("localhost", 1910);
  c->connect();
  c->setNoDelay(true);

  // Start thread
  c->start();

  PluginTool *pt = new PluginTool(&argp, c);

  SignalManager::register_handler(SIGINT, pt);
  pt->run();

  SignalManager::finalize();

  c->cancel();
  c->join();

  delete pt;
  delete c;

  return 0;
}
