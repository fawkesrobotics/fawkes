
/***************************************************************************
 *  ballposlog_plugin.cpp - Fawkes ball position log plugin for demonstration
 *
 *  Created: Thu Jan 24 16:58:45 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include "plugin.h"
#include "thread.h"

/** @class BallPosLogPlugin "plugin.h"
 * Simple ball position logger plugin.
 * This plugin exists for demonstration purposes. It is part of the Fawkes
 * introductory talk on January 25th 2008 or AG RoboCup.
 *
 * @author Tim Niemueller
 */

using namespace fawkes;

/** Constructor.
 * @param config Fawkes configuration
 */
BallPosLogPlugin::BallPosLogPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new BallPosLogThread());
}

PLUGIN_DESCRIPTION("Example that logs ball position")
EXPORT_PLUGIN(BallPosLogPlugin)

