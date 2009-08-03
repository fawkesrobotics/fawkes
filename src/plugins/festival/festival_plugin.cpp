
/***************************************************************************
 *  festival_plugin.cpp - Fawkes Festival Plugin
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

#include "festival_plugin.h"
#include "synth_thread.h"

using namespace fawkes;

/** @class FestivalPlugin "festival_plugin.h"
 * Festival plugin for Fawkes.
 * This plugin integrates Festival into Fawkes, allowing for simple and fast
 * speech synthesis.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
FestivalPlugin::FestivalPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new FestivalSynthThread());
}


PLUGIN_DESCRIPTION("Festival speech synthesis integration")
EXPORT_PLUGIN(FestivalPlugin)

