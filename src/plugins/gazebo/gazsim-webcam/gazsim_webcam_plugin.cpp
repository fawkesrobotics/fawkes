/***************************************************************************
 *  gazsim_webcam_plugin.cpp - Plugin simulates a Webcam in Gazebo and
 *                             provides a shared memory buffer
 *
 *  Created: Sat Sep 21 17:34:11 2013
 *  Copyright  2013 Frederik Zwilling
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

#include "gazsim_webcam_thread.h"

using namespace fawkes;

/** Plugin to simulate a webcam in Gazebo
 *
 *
 * @author Frederik Zwilling
 */
class GazsimWebcamPlugin : public fawkes::Plugin
{
 public:

  /** Constructor.
   * @param config Fawkes configuration
   */
  GazsimWebcamPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new WebcamSimThread());
  }
};

PLUGIN_DESCRIPTION("Simulation of a Webcam in Gazebo")
EXPORT_PLUGIN(GazsimWebcamPlugin)
