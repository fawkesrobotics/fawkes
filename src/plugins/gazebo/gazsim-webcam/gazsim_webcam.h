/***************************************************************************
 *  gazsim_webcam.h - Class to simulate a single webcam from gazebo
 *
 *  Created: Mon Mar 16 19:38:43 2015
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

#ifndef __PLUGINS_GAZSIM_WEBCAM_H_
#define __PLUGINS_GAZSIM_WEBCAM_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <string.h>
#include <config/config.h>

#include <fvutils/ipc/shm_image.h>
#include <boost/circular_buffer.hpp>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


class GazsimWebcam
{
 public:
  GazsimWebcam(std::string shm_id,
	       gazebo::transport::NodePtr gazebo_world_node,
	       fawkes::Configuration* config);
  ~GazsimWebcam();

 private:  
  //Subscriber to receive webcam data from gazebo
  gazebo::transport::SubscriberPtr webcam_sub_;

  //handler function for incoming webcam data messages
  void on_webcam_data_msg(ConstImageStampedPtr &msg);

  //shared memory buffer
  firevision::SharedMemoryImageBuffer *shm_buffer_;
  //reference to the buffer of shm_buffer_
  unsigned char *buffer_;

  //config values
  //topic name of the gazebo message publisher
  std::string topic_name_;
  double width_, height_;
  //id of the shared memory object
  std::string shm_id_;
  std::string frame_;
  firevision::colorspace_t format_from_;
  firevision::colorspace_t format_to_;
};

#endif
