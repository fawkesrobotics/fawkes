/***************************************************************************
 *  gazsim_webcam_plugin.cpp - Plugin simulates a Webcam in Gazebo and
 *                             provides a shared memory buffer
 *
 *  Created: Sat Sep 21 17:35:27 2013
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

#ifndef __PLUGINS_GAZSIM_WEBCAM_THREAD_H_
#define __PLUGINS_GAZSIM_WEBCAM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/gazebo/aspect/gazebo.h>
#include <string.h>

#include <fvutils/ipc/shm_image.h>
#include <boost/circular_buffer.hpp>

//from Gazebo
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/transport/transport.hh>


class WebcamSimThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::GazeboAspect
{
 public:
  WebcamSimThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

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
