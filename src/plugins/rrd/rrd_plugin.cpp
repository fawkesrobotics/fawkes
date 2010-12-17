
/***************************************************************************
 *  rrd_plugin.h - Fawkes RRD Plugin
 *
 *  Created: Sun Dec 05 23:22:23 2010 (Steelers vs. Baltimore, Touchdown!)
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

#include "rrd_plugin.h"
#include "rrd_thread.h"

using namespace fawkes;

/** @class RRDPlugin <plugins/rrd/rrd_plugin.h>
 * RRD manager plugin.
 * This plugin provides access to RRD for other Fawkes plugins. It manages
 * data written to the RRD, creates the database if necessary, and handles
 * concurrent graphing.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
RRDPlugin::RRDPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new RRDThread());
}


PLUGIN_DESCRIPTION("RRD management and graphing plugin")
EXPORT_PLUGIN(RRDPlugin)
