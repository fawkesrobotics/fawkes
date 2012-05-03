
/***************************************************************************
 *  robotino_pcl_plugin.cpp - Plugin to export IR sensor point cloud
 *
 *  Created: Mon Mar 26 14:03:55 2012
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

#include "ir_pcl_thread.h"

using namespace fawkes;

/** Plugin to provide Robotino IR sensor as point cloud
 * @author Tim Niemueller
 */
class RobotinoIrPclPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RobotinoIrPclPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new RobotinoIrPclThread());
  }
};

PLUGIN_DESCRIPTION("Robotino IR sensor point cloud")
EXPORT_PLUGIN(RobotinoIrPclPlugin)
