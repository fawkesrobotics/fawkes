/***************************************************************************
 *  imu_plugin.cpp - Publish IMU data to ROS
 *
 *  Created: Mon 03 Apr 2017 13:21:33 CEST 13:21
 *  Copyright  2017  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "imu_thread.h"

using namespace fawkes;

/** Plugin to publish IMU data to ROS.
 * @author Till Hofmann
 */
class RosIMUPlugin
: public fawkes::Plugin
{
 public:
   /** Constructor.
    * @param config Fawkes configuration
    */
   RosIMUPlugin(Configuration *config)
     : Plugin(config)
   {
     thread_list.push_back(new RosIMUThread());
   }
};

PLUGIN_DESCRIPTION("ROS IMU Publisher Plugin")
EXPORT_PLUGIN(RosIMUPlugin)
