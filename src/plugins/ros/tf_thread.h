
/***************************************************************************
 *  tf_thread.h - Thread to exchange transforms
 *
 *  Created: Wed Oct 26 00:50:12 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROS_TF_THREAD_H_
#define __PLUGINS_ROS_TF_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/tf.h>
#include <plugins/ros/aspect/ros.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <interfaces/TransformInterface.h>
#include <core/threading/mutex.h>

#include <list>
#include <queue>

// from ROS
#include <ros/common.h>
#include <ros/node_handle.h>
#include <tf/tfMessage.h>
#ifdef HAVE_TF2_MSGS
#  include <tf2_msgs/TFMessage.h>
#endif

class RosTfThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect,
  public fawkes::ROSAspect,
  public fawkes::BlackBoardInterfaceObserver,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  RosTfThread();
  virtual ~RosTfThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  // for BlackBoardInterfaceObserver
  virtual void bb_interface_created(const char *type, const char *id) throw();

  // for BlackBoardInterfaceListener
  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();
  virtual void bb_interface_reader_removed(fawkes::Interface *interface,
                                           unsigned int instance_serial) throw();

 private:
  void tf_message_cb(const ros::MessageEvent<::tf::tfMessage const> &msg_evt);
#ifdef HAVE_TF2_MSGS
  void tf_message_cb_dynamic(const ros::MessageEvent<tf2_msgs::TFMessage const> &msg_evt);
  void tf_message_cb_static(const ros::MessageEvent<tf2_msgs::TFMessage const> &msg_evt);
  void tf_message_cb(const ros::MessageEvent<tf2_msgs::TFMessage const> &msg_evt, bool static_tf);
#endif

  void conditional_close(fawkes::Interface *interface) throw();
  void publish_static_transforms_to_ros();
  void publish_transform_to_fawkes(const geometry_msgs::TransformStamped &ts,
                                   bool static_tf = false);
  geometry_msgs::TransformStamped create_transform_stamped(fawkes::TransformInterface *tfif,
                                                           const fawkes::Time *time = NULL);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  bool __cfg_use_tf2;
  float __cfg_update_interval;


  std::list<std::string>                  __ros_frames;
  std::list<fawkes::TransformInterface *> __tfifs;

  ros::Subscriber __sub_tf;
  ros::Subscriber __sub_static_tf;
  ros::Publisher  __pub_tf;
  ros::Publisher  __pub_static_tf;

  fawkes::Mutex *__tf_msg_queue_mutex;
  unsigned int __active_queue;
  std::queue<::tf::tfMessage::ConstPtr>     __tf_msg_queues[2];
#ifdef HAVE_TF2_MSGS
  std::queue<std::pair<bool, tf2_msgs::TFMessage::ConstPtr> > __tf2_msg_queues[2];
#endif

  fawkes::Mutex *__seq_num_mutex;
  unsigned int   __seq_num;

  fawkes::Time *__last_update;

};

#endif
