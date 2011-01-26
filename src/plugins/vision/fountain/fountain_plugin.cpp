
/***************************************************************************
 *  fountain_plugin.cpp - FireVision Fountain Plugin
 *
 *  Created: Fri Nov 16 11:29:48 2007 (Ella in lab for the first time)
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

#include "fountain_plugin.h"
#include "fountain_thread.h"

using namespace fawkes;

/** @class FvFountainPlugin "fountain_plugin.h"
 * FireVision Fountain Plugin.
 * This is the FireVision fountain plugin. It is a simple plugin that will
 * start a FuseServer that will then fulfill remote requests for internal
 * vision data.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param config Fawkes configuration
 */
FvFountainPlugin::FvFountainPlugin(Configuration *config)
  : Plugin(config)
{
  thread_list.push_back(new FountainThread());
}

PLUGIN_DESCRIPTION("Provides access to images, colormaps etc. via network")
EXPORT_PLUGIN(FvFountainPlugin)
