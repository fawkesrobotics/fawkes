
/***************************************************************************
 *  rrd_example_plugin.cpp - Fawkes RRD Example Plugin
 *
 *  Created: Mon Jan 10 00:06:11 2011
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

#include "rrd_example_plugin.h"
#include "rrd_example_thread.h"

using namespace fawkes;

/** @class RRDExamplePlugin "rrd_example_plugin.h"
 * RRD Example plugin.
 * Simple plugin to show how to create RRD graphs.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
RRDExamplePlugin::RRDExamplePlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new RRDExampleThread());
}


PLUGIN_DESCRIPTION("RRD example graph plugin.")
EXPORT_PLUGIN(RRDExamplePlugin)
