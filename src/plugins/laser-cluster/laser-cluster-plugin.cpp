
/***************************************************************************
 *  laser-cluster-plugin.cpp - Detect a cluster from 2D laser data
 *
 *  Created: Sun Apr 21 01:15:54 2013
 *  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
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

#include "laser-cluster-thread.h"

using namespace fawkes;

/** Plugin to detect a cluster in 2D laser data.
 * @author Tim Niemueller
 */
class LaserClusterPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  LaserClusterPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new LaserClusterThread());
  }
};

PLUGIN_DESCRIPTION("Detect cluster in 2D laser data")
EXPORT_PLUGIN(LaserClusterPlugin)
