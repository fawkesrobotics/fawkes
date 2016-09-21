/***************************************************************************
 *  gazsim_depthcam_plugin.cpp - Plugin simulates a Depthcam in Gazebo and
 *                             provides the point cloud
 *
 *  Created: Fri Feb 19 20:57:59 2016
 *  Copyright  2016 Frederik Zwilling
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

#include "gazsim_depthcam_thread.h"

using namespace fawkes;

/** Plugin to simulate a depthcam in Gazebo
 *
 *
 * @author Frederik Zwilling
 */
class GazsimDepthcamPlugin : public fawkes::Plugin
{
 public:

  /** Constructor.
   * @param config Fawkes configuration
   */
  GazsimDepthcamPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new DepthcamSimThread());
  }
};

PLUGIN_DESCRIPTION("Simulation of a Depthcam in Gazebo")
EXPORT_PLUGIN(GazsimDepthcamPlugin)
