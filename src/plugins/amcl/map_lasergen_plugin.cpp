
/***************************************************************************
 *  map_lasergen_plugin.cpp - Generate laser scans from map position
 *
 *  Created: Thu Aug 23 18:32:44 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#include <core/plugin.h>

#include "map_lasergen_thread.h"

using namespace fawkes;

/** Laser data from map generator plugin.
 * @author Tim Niemueller
 */
class MapLaserGenPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  MapLaserGenPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new MapLaserGenThread());
  }
};

PLUGIN_DESCRIPTION("Generate laser data from map position")
EXPORT_PLUGIN(MapLaserGenPlugin)
