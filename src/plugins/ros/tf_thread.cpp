
/***************************************************************************
 *  tf_thread.cpp - Thread to exchange transforms
 *
 *  Created: Wed Oct 26 01:02:59 2011
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

#include "tf_thread.h"

#include <core/threading/mutex_locker.h>
#include <ros/this_node.h>

using namespace fawkes;

/** @class RosTfThread "tf_thread.h"
 * Thread to exchange transforms between Fawkes and ROS.
 * This threads connects to Fawkes and ROS to read and write transforms.
 * Transforms received on one end are republished to the other side. To
 * Fawkes new frames are published during the sensor hook.
 * @author Tim Niemueller
 */

/** Constructor. */
RosTfThread::RosTfThread()
  : Thread("RosTfThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    TransformAspect(TransformAspect::DEFER_PUBLISHER),
    BlackBoardInterfaceListener("RosTfThread")
{
  __tf_msg_queue_mutex = new Mutex();
  __seq_num_mutex = new Mutex();
  __last_update = new Time();
}

/** Destructor. */
RosTfThread::~RosTfThread()
{
  delete __tf_msg_queue_mutex;
  delete __seq_num_mutex;
  delete __last_update;
}



void
RosTfThread::init()
{
  __active_queue = 0;
  __seq_num = 0;
  __last_update->set_clock(clock);
  __last_update->set_time(0, 0);

  __cfg_use_tf2 = config->get_bool("/ros/tf/use_tf2");
  __cfg_update_interval = 1.0;
  try  {
	  __cfg_update_interval = config->get_float("/ros/tf/static-update-interval");
  } catch (Exception &e) {} // ignored, use default

  // Must do that before registering listener because we might already
  // get events right away
  if (__cfg_use_tf2) {
#ifndef HAVE_TF2_MSGS
	  throw Exception("tf2 enabled in config but not available at compile time");
#else
	  __sub_tf = rosnode->subscribe("tf", 100, &RosTfThread::tf_message_cb_dynamic, this);
	  __sub_static_tf = rosnode->subscribe("tf_static", 100, &RosTfThread::tf_message_cb_static, this);
	  __pub_tf = rosnode->advertise< tf2_msgs::TFMessage >("tf", 100);
	  __pub_static_tf = rosnode->advertise< tf2_msgs::TFMessage >("tf_static", 100, /* latch */ true);
#endif
  } else {
	  __sub_tf = rosnode->subscribe("tf", 100, &RosTfThread::tf_message_cb, this);
	  __pub_tf = rosnode->advertise< ::tf::tfMessage >("tf", 100);
  }

  __tfifs = blackboard->open_multiple_for_reading<TransformInterface>("/tf*");
  std::list<TransformInterface *>::iterator i;
  for (i = __tfifs.begin(); i != __tfifs.end(); ++i) {
	  bbil_add_data_interface(*i);
	  bbil_add_reader_interface(*i);
	  bbil_add_writer_interface(*i);
  }
  blackboard->register_listener(this);

  publish_static_transforms_to_ros();
  
  bbio_add_observed_create("TransformInterface", "/tf*");
  blackboard->register_observer(this);
}


void
RosTfThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->unregister_observer(this);

  __sub_tf.shutdown();
  __pub_tf.shutdown();

  std::list<TransformInterface *>::iterator i;
  for (i = __tfifs.begin(); i != __tfifs.end(); ++i) {
    blackboard->close(*i);
  }
  __tfifs.clear();
}


void
RosTfThread::loop()
{
  __tf_msg_queue_mutex->lock();
  unsigned int queue = __active_queue;
  __active_queue = 1 - __active_queue;
  __tf_msg_queue_mutex->unlock();

  if (__cfg_use_tf2) {
#ifdef HAVE_TF2_MSGS
	  while (! __tf2_msg_queues[queue].empty()) {
		  const std::pair<bool, tf2_msgs::TFMessage::ConstPtr> &q = __tf2_msg_queues[queue].front();
		  const tf2_msgs::TFMessage::ConstPtr &msg = q.second;
		  const size_t tsize = msg->transforms.size();
		  for (size_t i = 0; i < tsize; ++i) {
			  publish_transform_to_fawkes(msg->transforms[i], q.first);
		  }
		  __tf2_msg_queues[queue].pop();
	  }
#endif
  } else {
	  while (! __tf_msg_queues[queue].empty()) {
		  const ::tf::tfMessage::ConstPtr &msg = __tf_msg_queues[queue].front();
		  const size_t tsize = msg->transforms.size();
		  for (size_t i = 0; i < tsize; ++i) {
			  geometry_msgs::TransformStamped ts = msg->transforms[i];
			  if (! ts.header.frame_id.empty() && ts.header.frame_id[0] == '/') {
				  ts.header.frame_id = ts.header.frame_id.substr(1);
			  }
			  if (! ts.child_frame_id.empty() && ts.child_frame_id[0] == '/') {
				  ts.child_frame_id = ts.child_frame_id.substr(1);
			  }
			  publish_transform_to_fawkes(ts);
		  }
		  __tf_msg_queues[queue].pop();
	  }

	  fawkes::Time now(clock);
	  if ((now - __last_update) > __cfg_update_interval) {
		  __last_update->stamp();

		  publish_static_transforms_to_ros();
	  }
  }
}


void
RosTfThread::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
  TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
  if (! tfif) return;

  tfif->read();

  if (__cfg_use_tf2 && tfif->is_static_transform()) {
	  publish_static_transforms_to_ros();
  } else {
	  if (__cfg_use_tf2) {
#ifdef HAVE_TF2_MSGS
		  tf2_msgs::TFMessage tmsg;
		  tmsg.transforms.push_back(create_transform_stamped(tfif));
		  __pub_tf.publish(tmsg);
#endif
	  } else {
		  ::tf::tfMessage tmsg;
		  if (tfif->is_static_transform()) {
			  // date time stamps slightly into the future so they are valid
			  // for longer and need less frequent updates.
			  fawkes::Time timestamp = fawkes::Time(clock) + (__cfg_update_interval * 1.1);

			  tmsg.transforms.push_back(create_transform_stamped(tfif, &timestamp));
		  } else {
			  tmsg.transforms.push_back(create_transform_stamped(tfif));
		  }
		  __pub_tf.publish(tmsg);
	  }
  }
}


void
RosTfThread::bb_interface_created(const char *type, const char *id) throw()
{
  if (strncmp(type, "TransformInterface", __INTERFACE_TYPE_SIZE) != 0)  return;

  for (const auto &f : __ros_frames) {
	  // ignore interfaces that we publish ourself
	  if (f == id)  return;
  }
  
  TransformInterface *tfif;
  try {
    //logger->log_info(name(), "Opening %s:%s", type, id);
    tfif = blackboard->open_for_reading<TransformInterface>(id);
  } catch (Exception &e) {
    // ignored
    logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
    return;
  }

  try {
    bbil_add_data_interface(tfif);
    bbil_add_reader_interface(tfif);
    bbil_add_writer_interface(tfif);
    blackboard->update_listener(this);
    __tfifs.push_back(tfif);
  } catch (Exception &e) {
    blackboard->close(tfif);
    logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
    return;
  }
}

void
RosTfThread::bb_interface_writer_removed(fawkes::Interface *interface,
                                         unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
RosTfThread::bb_interface_reader_removed(fawkes::Interface *interface,
                                         unsigned int instance_serial) throw()
{
  conditional_close(interface);
}

void
RosTfThread::conditional_close(Interface *interface) throw()
{
  // Verify it's a TransformInterface
  TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
  if (! tfif) return;
  
  std::list<TransformInterface *>::iterator i;
  for (i = __tfifs.begin(); i != __tfifs.end(); ++i) {
    if (*interface == **i) {
      if (! interface->has_writer() && (interface->num_readers() == 1)) {
        // It's only us
        logger->log_info(name(), "Last on %s, closing", interface->uid());
        bbil_remove_data_interface(*i);
        bbil_remove_reader_interface(*i);
        bbil_remove_writer_interface(*i);
        blackboard->update_listener(this);
        blackboard->close(*i);
        __tfifs.erase(i);
        break;
      }
    }
  }
}


geometry_msgs::TransformStamped
RosTfThread::create_transform_stamped(TransformInterface *tfif, const Time *time)
{
  double *translation = tfif->translation();
  double *rotation = tfif->rotation();
  if (! time)  time = tfif->timestamp();

  geometry_msgs::Vector3 t;
  t.x = translation[0]; t.y = translation[1]; t.z = translation[2];
  geometry_msgs::Quaternion r;
  r.x = rotation[0]; r.y = rotation[1]; r.z = rotation[2]; r.w = rotation[3];
  geometry_msgs::Transform tr;
  tr.translation = t;
  tr.rotation = r;

  geometry_msgs::TransformStamped ts;
  __seq_num_mutex->lock();
  ts.header.seq = ++__seq_num;
  __seq_num_mutex->unlock();
  ts.header.stamp = ros::Time(time->get_sec(), time->get_nsec());
  ts.header.frame_id = tfif->frame();
  ts.child_frame_id = tfif->child_frame();
  ts.transform = tr;

  return ts;
}

void
RosTfThread::publish_static_transforms_to_ros()
{
	std::list<fawkes::TransformInterface *>::iterator t;
	fawkes::Time now(clock);

	if (__cfg_use_tf2) {
#ifdef HAVE_TF2_MSGS
		tf2_msgs::TFMessage tmsg;
		for (t = __tfifs.begin(); t != __tfifs.end(); ++t) {
			fawkes::TransformInterface *tfif = *t;
			tfif->read();
			if (tfif->is_static_transform()) {
				tmsg.transforms.push_back(create_transform_stamped(tfif, &now));
			}
		}
		__pub_static_tf.publish(tmsg);
#endif
	} else {
		// date time stamps slightly into the future so they are valid
		// for longer and need less frequent updates.
		fawkes::Time timestamp = now + (__cfg_update_interval * 1.1);

		::tf::tfMessage tmsg;
		for (t = __tfifs.begin(); t != __tfifs.end(); ++t) {
			fawkes::TransformInterface *tfif = *t;
			tfif->read();
			if (tfif->is_static_transform()) {
				tmsg.transforms.push_back(create_transform_stamped(tfif, &timestamp));
			}
		}
		__pub_tf.publish(tmsg);
	}
}


void
RosTfThread::publish_transform_to_fawkes(const geometry_msgs::TransformStamped &ts, bool static_tf)
{
	const geometry_msgs::Vector3 &t = ts.transform.translation;
	const geometry_msgs::Quaternion &r = ts.transform.rotation;

	fawkes::Time time(ts.header.stamp.sec, ts.header.stamp.nsec / 1000);

	fawkes::tf::Transform tr(fawkes::tf::Quaternion(r.x, r.y, r.z, r.w),
	                         fawkes::tf::Vector3(t.x, t.y, t.z));
	fawkes::tf::StampedTransform
		st(tr, time, ts.header.frame_id, ts.child_frame_id);

	if (tf_publishers.find(ts.child_frame_id) == tf_publishers.end()) {
		try {
			__ros_frames.push_back(std::string("/tf/") + ts.child_frame_id);
			tf_add_publisher("%s", ts.child_frame_id.c_str());
			tf_publishers[ts.child_frame_id]->send_transform(st, static_tf);
		} catch (Exception &e) {
			__ros_frames.pop_back();
			logger->log_warn(name(), "Failed to create Fawkes transform publisher for frame %s from ROS",
			                 ts.child_frame_id.c_str());
			logger->log_warn(name(), e);
		}
	} else {
		tf_publishers[ts.child_frame_id]->send_transform(st, static_tf);
	}
}


/** Callback function for ROS tf message subscription.
 * @param msg incoming message
 */
void
RosTfThread::tf_message_cb(const ros::MessageEvent<::tf::tfMessage const> &msg_evt)
{
  MutexLocker lock(__tf_msg_queue_mutex);

  const ::tf::tfMessage::ConstPtr &msg = msg_evt.getConstMessage();
  std::string authority = msg_evt.getPublisherName();

  if (authority == "") {
    logger->log_warn(name(), "Message received without callerid");
  } else if (authority != ros::this_node::getName()) {
    __tf_msg_queues[__active_queue].push(msg);
  }
}

#ifdef HAVE_TF2_MSGS
void
RosTfThread::tf_message_cb_static(const ros::MessageEvent<tf2_msgs::TFMessage const> &msg_evt)
{
	tf_message_cb(msg_evt, true);
}

void
RosTfThread::tf_message_cb_dynamic(const ros::MessageEvent<tf2_msgs::TFMessage const> &msg_evt)
{
	tf_message_cb(msg_evt, false);
}

void
RosTfThread::tf_message_cb(const ros::MessageEvent<tf2_msgs::TFMessage const> &msg_evt, bool static_tf)
{
  MutexLocker lock(__tf_msg_queue_mutex);

  const tf2_msgs::TFMessage::ConstPtr &msg = msg_evt.getConstMessage();
  std::string authority = msg_evt.getPublisherName();

  if (authority == "") {
    logger->log_warn(name(), "Message received without callerid");
  } else if (authority != ros::this_node::getName()) {
	  __tf2_msg_queues[__active_queue].push(std::make_pair(static_tf, msg));
  }
}
#endif
