
/***************************************************************************
 *  flite_plugin.cpp - Fawkes Flite Plugin
 *
 *  Created: Tue Oct 28 14:30:19 2008
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

#include "flite_plugin.h"
#include "synth_thread.h"

using namespace fawkes;

/** @class FlitePlugin "flite_plugin.h"
 * flite plugin for Fawkes.
 * This plugin integrates Flite into Fawkes, allowing for simple and fast
 * speech synthesis.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
FlitePlugin::FlitePlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new FliteSynthThread());
}


PLUGIN_DESCRIPTION("Flite speech synthesis integration")
EXPORT_PLUGIN(FlitePlugin)
