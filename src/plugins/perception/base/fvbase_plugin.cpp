
/***************************************************************************
 *  fvbase_plugin.cpp - FireVision Base Plugin
 *
 *  Created: Tue May 29 16:35:48 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#include "fvbase_plugin.h"
#include "base_thread.h"

using namespace fawkes;

/** @class FvBasePlugin "fvbase_plugin.h"
 * FireVision Base Plugin
 * This is the FireVision base plugin. It provides access to low-level
 * facilities like cameras and is responsible for the timing of vision
 * threads.
 *
 * It acts as a vision provider to vision threads.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
FvBasePlugin::FvBasePlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new FvBaseThread());
}

PLUGIN_DESCRIPTION("FireVision Base provides access to camera and handles timing")
EXPORT_PLUGIN(FvBasePlugin)
