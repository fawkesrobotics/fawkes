/***************************************************************************
 *  gazsim_webcam_plugin.cpp - Plugin simulates a Webcam in Gazebo and
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
 * Thread simulates a webcam in Gazebo
 * @author Frederik Zwilling
 */

/** Constructor. */
WebcamSimThread::WebcamSimThread()
  : Thread("WebcamSimThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
  shm_buffer_ = NULL;
}

void WebcamSimThread::init()
{
  logger->log_debug(name(), "Initializing Simulation of the Webcam");

  //read config values
  std::string robot_name = config->get_string("/gazsim/robot-name");
  topic_name_ = ("~/" 
		 + robot_name
		 + config->get_string("/gazsim/webcam/topic-suffix"));
  width_ = config->get_float("/gazsim/webcam/width");
  height_ = config->get_float("/gazsim/webcam/height");
  shm_id_ = robot_name + "/" + config->get_string("/gazsim/webcam/shm-image-id");
  frame_ = config->get_string("/gazsim/webcam/frame");

  format_from_ = firevision::RGB;
  format_to_ = firevision::YUV422_PLANAR;

  //subscribing to gazebo publisher
  //the messages are published by the sensor itself and not by a robot plugin
  //therefore we have to use the world node
  webcam_sub_ = gazebo_world_node->Subscribe(topic_name_, &WebcamSimThread::on_webcam_data_msg, this);

  //initialize shared memory image buffer
  shm_buffer_ = new firevision::SharedMemoryImageBuffer( shm_id_.c_str(),
							 format_to_,
							 width_,
							 height_
							 );
  if (!shm_buffer_->is_valid()) {
    throw fawkes::Exception("Shared memory segment not valid");
  }
  shm_buffer_->set_frame_id(frame_.c_str());
  buffer_ = shm_buffer_->buffer();
  //enable locking
  shm_buffer_->add_semaphore();
}

void WebcamSimThread::finalize()
{
  delete this->shm_buffer_;
}

void WebcamSimThread::loop()
{
  //The interesting stuff happens in the on_webcam_data_msg handler
}

void WebcamSimThread::on_webcam_data_msg(ConstImageStampedPtr &msg)
{
  //logger->log_info(name(), "Got new Webcam data.");
  
  //convert image data and write it in the shared memory buffer
  //lock the shm so noone can read a half written image
  shm_buffer_->lock_for_write();
  convert(format_from_,  format_to_,
 	  (const unsigned char*) msg->image().data().data(),   buffer_,
	  width_, height_);
  shm_buffer_->unlock();
}
