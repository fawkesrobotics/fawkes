/***************************************************************************
 *  gazsim_webcam_plugin.cpp - Plugin simulates Webcams in Gazebo and
 *                             provides a shared memory buffer
 *
 *  Created: Sat Sep 21 17:37:42 2013
 *  Copyright  2013  Frederik Zwilling
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

#include "gazsim_webcam_thread.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <utils/math/angle.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <aspect/logging.h>

#include <fvutils/color/conversions.h>

using namespace fawkes;
using namespace gazebo;

/** @class WebcamSimThread "gazsim_webcam_thread.h"
 * Thread simulates a number of webcams in Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
WebcamSimThread::WebcamSimThread()
  : Thread("WebcamSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

void WebcamSimThread::init()
{
  logger->log_debug(name(), "Initializing Simulation of the Webcams");
  shm_ids_ = config->get_strings("/gazsim/webcam/shm-image-ids");

  for (std::vector<std::string>::iterator it = shm_ids_.begin(); it != shm_ids_.end(); ++it)
  {
    webcams_.push_back(new GazsimWebcam(*it, gazebo_world_node, config));
  }
}

void WebcamSimThread::finalize()
{
  for (std::vector<GazsimWebcam*>::iterator it = webcams_.begin(); it != webcams_.end(); ++it)
  {
    delete *it;
  }  
}

void WebcamSimThread::loop()
{
  //The interesting stuff happens in the callback of the webcams
}
