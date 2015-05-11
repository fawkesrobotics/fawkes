/***************************************************************************
 *  gazsim_webcam.cpp - Class to simulate a single webcam from gazebo
 *
 *  Created: Mon Mar 16 19:43:05 2015
 *  Copyright  2015  Frederik Zwilling
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

#include "gazsim_webcam.h"

#include <tf/types.h>
#include <stdio.h>
#include <math.h>
#include <utils/math/angle.h>

#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <fvutils/color/conversions.h>

using namespace fawkes;

/** @class GazsimWebcam "gazsim_webcam.h"
 * Simulates a single webcam in Gazebo
 * @author Frederik Zwilling
 */


/** Constructor.
 * The GazsimWebcam object simulates a single webcam in Gazebo
 * @param shm_id The shared memory id the simulated webcam should write to and the prefix of the config values for this camera
 * @param gazebo_world_node gazebo world node to register subscribers
 * @param config config object to access config values
 */
GazsimWebcam::GazsimWebcam(std::string shm_id,
			     gazebo::transport::NodePtr gazebo_world_node,
			     Configuration* config)
{
  shm_buffer_ = NULL;
  //read config values
  std::string robot_name = config->get_string("/gazsim/robot-name");
  shm_id_ = robot_name + "/" + shm_id;
  topic_name_ = ("~/" 
		 + robot_name
                 + config->get_string((std::string("/gazsim/webcam/topic-suffixes/")
		                        + shm_id).c_str()));
  width_ = config->get_float((std::string("/gazsim/webcam/widths/") + shm_id).c_str());
  height_ = config->get_float((std::string("/gazsim/webcam/heights/") + shm_id).c_str());
  frame_ = config->get_string((std::string("/gazsim/webcam/frames/") + shm_id).c_str());

  format_from_ = firevision::RGB;
  format_to_ = firevision::YUV422_PLANAR;

  //subscribing to gazebo publisher
  //the messages are published by the sensor itself and not by a robot plugin
  //therefore we have to use the world node
  webcam_sub_ = gazebo_world_node->Subscribe(topic_name_, &GazsimWebcam::on_webcam_data_msg, this);

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

GazsimWebcam::~GazsimWebcam()
{
  delete this->shm_buffer_;
}

void GazsimWebcam::on_webcam_data_msg(ConstImageStampedPtr &msg)
{  
  //convert image data and write it in the shared memory buffer
  //lock the shm so noone can read a half written image
  shm_buffer_->lock_for_write();
  convert(format_from_,  format_to_,
 	  (const unsigned char*) msg->image().data().data(),   buffer_,
	  width_, height_);
  shm_buffer_->unlock();
}
