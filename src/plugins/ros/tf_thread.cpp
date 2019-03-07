
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
	tf_msg_queue_mutex_ = new Mutex();
	seq_num_mutex_      = new Mutex();
	last_update_        = new Time();
}

/** Destructor. */
RosTfThread::~RosTfThread()
{
	delete tf_msg_queue_mutex_;
	delete seq_num_mutex_;
	delete last_update_;
}

void
RosTfThread::init()
{
	active_queue_ = 0;
	seq_num_      = 0;
	last_update_->set_clock(clock);
	last_update_->set_time(0, 0);

	cfg_use_tf2_         = config->get_bool("/ros/tf/use_tf2");
	cfg_update_interval_ = 1.0;
	try {
		cfg_update_interval_ = config->get_float("/ros/tf/static-update-interval");
	} catch (Exception &e) {
	} // ignored, use default

	// Must do that before registering listener because we might already
	// get events right away
	if (cfg_use_tf2_) {
#ifndef HAVE_TF2_MSGS
		throw Exception("tf2 enabled in config but not available at compile time");
#else
		sub_tf_        = rosnode->subscribe("tf", 100, &RosTfThread::tf_message_cb_dynamic, this);
		sub_static_tf_ = rosnode->subscribe("tf_static", 100, &RosTfThread::tf_message_cb_static, this);
		pub_tf_        = rosnode->advertise<tf2_msgs::TFMessage>("tf", 100);
		pub_static_tf_ = rosnode->advertise<tf2_msgs::TFMessage>("tf_static", 100, /* latch */ true);
#endif
	} else {
		sub_tf_ = rosnode->subscribe("tf", 100, &RosTfThread::tf_message_cb, this);
		pub_tf_ = rosnode->advertise<::tf::tfMessage>("tf", 100);
	}

	tfifs_ = blackboard->open_multiple_for_reading<TransformInterface>("/tf*");
	std::list<TransformInterface *>::iterator i;
	for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
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

	sub_tf_.shutdown();
	pub_tf_.shutdown();

	std::list<TransformInterface *>::iterator i;
	for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
		blackboard->close(*i);
	}
	tfifs_.clear();
}

void
RosTfThread::loop()
{
	tf_msg_queue_mutex_->lock();
	unsigned int queue = active_queue_;
	active_queue_      = 1 - active_queue_;
	tf_msg_queue_mutex_->unlock();

	if (cfg_use_tf2_) {
#ifdef HAVE_TF2_MSGS
		while (!tf2_msg_queues_[queue].empty()) {
			const std::pair<bool, tf2_msgs::TFMessage::ConstPtr> &q     = tf2_msg_queues_[queue].front();
			const tf2_msgs::TFMessage::ConstPtr &                 msg   = q.second;
			const size_t                                          tsize = msg->transforms.size();
			for (size_t i = 0; i < tsize; ++i) {
				publish_transform_to_fawkes(msg->transforms[i], q.first);
			}
			tf2_msg_queues_[queue].pop();
		}
#endif
	} else {
		while (!tf_msg_queues_[queue].empty()) {
			const ::tf::tfMessage::ConstPtr &msg   = tf_msg_queues_[queue].front();
			const size_t                     tsize = msg->transforms.size();
			for (size_t i = 0; i < tsize; ++i) {
				geometry_msgs::TransformStamped ts = msg->transforms[i];
				if (!ts.header.frame_id.empty() && ts.header.frame_id[0] == '/') {
					ts.header.frame_id = ts.header.frame_id.substr(1);
				}
				if (!ts.child_frame_id.empty() && ts.child_frame_id[0] == '/') {
					ts.child_frame_id = ts.child_frame_id.substr(1);
				}
				publish_transform_to_fawkes(ts);
			}
			tf_msg_queues_[queue].pop();
		}

		fawkes::Time now(clock);
		if ((now - last_update_) > cfg_update_interval_) {
			last_update_->stamp();

			publish_static_transforms_to_ros();
		}
	}
}

void
RosTfThread::bb_interface_data_changed(fawkes::Interface *interface) throw()
{
	TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
	if (!tfif)
		return;

	tfif->read();

	if (cfg_use_tf2_ && tfif->is_static_transform()) {
		publish_static_transforms_to_ros();
	} else {
		if (cfg_use_tf2_) {
#ifdef HAVE_TF2_MSGS
			tf2_msgs::TFMessage tmsg;
			tmsg.transforms.push_back(create_transform_stamped(tfif));
			pub_tf_.publish(tmsg);
#endif
		} else {
			::tf::tfMessage tmsg;
			if (tfif->is_static_transform()) {
				// date time stamps slightly into the future so they are valid
				// for longer and need less frequent updates.
				fawkes::Time timestamp = fawkes::Time(clock) + (cfg_update_interval_ * 1.1);

				tmsg.transforms.push_back(create_transform_stamped(tfif, &timestamp));
			} else {
				tmsg.transforms.push_back(create_transform_stamped(tfif));
			}
			pub_tf_.publish(tmsg);
		}
	}
}

void
RosTfThread::bb_interface_created(const char *type, const char *id) throw()
{
	if (strncmp(type, "TransformInterface", INTERFACE_TYPE_SIZE_) != 0)
		return;

	for (const auto &f : ros_frames_) {
		// ignore interfaces that we publish ourself
		if (f == id)
			return;
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
		tfifs_.push_back(tfif);
	} catch (Exception &e) {
		blackboard->close(tfif);
		logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
		return;
	}
}

void
RosTfThread::bb_interface_writer_removed(fawkes::Interface *interface,
                                         unsigned int       instance_serial) throw()
{
	conditional_close(interface);
}

void
RosTfThread::bb_interface_reader_removed(fawkes::Interface *interface,
                                         unsigned int       instance_serial) throw()
{
	conditional_close(interface);
}

void
RosTfThread::conditional_close(Interface *interface) throw()
{
	// Verify it's a TransformInterface
	TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
	if (!tfif)
		return;

	std::list<TransformInterface *>::iterator i;
	for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
		if (*interface == **i) {
			if (!interface->has_writer() && (interface->num_readers() == 1)) {
				// It's only us
				logger->log_info(name(), "Last on %s, closing", interface->uid());
				bbil_remove_data_interface(*i);
				bbil_remove_reader_interface(*i);
				bbil_remove_writer_interface(*i);
				blackboard->update_listener(this);
				blackboard->close(*i);
				tfifs_.erase(i);
				break;
			}
		}
	}
}

geometry_msgs::TransformStamped
RosTfThread::create_transform_stamped(TransformInterface *tfif, const Time *time)
{
	double *translation = tfif->translation();
	double *rotation    = tfif->rotation();
	if (!time)
		time = tfif->timestamp();

	geometry_msgs::Vector3 t;
	t.x = translation[0];
	t.y = translation[1];
	t.z = translation[2];
	geometry_msgs::Quaternion r;
	r.x = rotation[0];
	r.y = rotation[1];
	r.z = rotation[2];
	r.w = rotation[3];
	geometry_msgs::Transform tr;
	tr.translation = t;
	tr.rotation    = r;

	geometry_msgs::TransformStamped ts;
	seq_num_mutex_->lock();
	ts.header.seq = ++seq_num_;
	seq_num_mutex_->unlock();
	ts.header.stamp    = ros::Time(time->get_sec(), time->get_nsec());
	ts.header.frame_id = tfif->frame();
	ts.child_frame_id  = tfif->child_frame();
	ts.transform       = tr;

	return ts;
}

void
RosTfThread::publish_static_transforms_to_ros()
{
	std::list<fawkes::TransformInterface *>::iterator t;
	fawkes::Time                                      now(clock);

	if (cfg_use_tf2_) {
#ifdef HAVE_TF2_MSGS
		tf2_msgs::TFMessage tmsg;
		for (t = tfifs_.begin(); t != tfifs_.end(); ++t) {
			fawkes::TransformInterface *tfif = *t;
			tfif->read();
			if (tfif->is_static_transform()) {
				tmsg.transforms.push_back(create_transform_stamped(tfif, &now));
			}
		}
		pub_static_tf_.publish(tmsg);
#endif
	} else {
		// date time stamps slightly into the future so they are valid
		// for longer and need less frequent updates.
		fawkes::Time timestamp = now + (cfg_update_interval_ * 1.1);

		::tf::tfMessage tmsg;
		for (t = tfifs_.begin(); t != tfifs_.end(); ++t) {
			fawkes::TransformInterface *tfif = *t;
			tfif->read();
			if (tfif->is_static_transform()) {
				tmsg.transforms.push_back(create_transform_stamped(tfif, &timestamp));
			}
		}
		pub_tf_.publish(tmsg);
	}
}

void
RosTfThread::publish_transform_to_fawkes(const geometry_msgs::TransformStamped &ts, bool static_tf)
{
	const geometry_msgs::Vector3 &   t = ts.transform.translation;
	const geometry_msgs::Quaternion &r = ts.transform.rotation;

	fawkes::Time time(ts.header.stamp.sec, ts.header.stamp.nsec / 1000);

	fawkes::tf::Transform        tr(fawkes::tf::Quaternion(r.x, r.y, r.z, r.w),
                           fawkes::tf::Vector3(t.x, t.y, t.z));
	fawkes::tf::StampedTransform st(tr, time, ts.header.frame_id, ts.child_frame_id);

	if (tf_publishers.find(ts.child_frame_id) == tf_publishers.end()) {
		try {
			ros_frames_.push_back(std::string("/tf/") + ts.child_frame_id);
			tf_add_publisher("%s", ts.child_frame_id.c_str());
			tf_publishers[ts.child_frame_id]->send_transform(st, static_tf);
		} catch (Exception &e) {
			ros_frames_.pop_back();
			logger->log_warn(name(),
			                 "Failed to create Fawkes transform publisher for frame %s from ROS",
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
	MutexLocker lock(tf_msg_queue_mutex_);

	const ::tf::tfMessage::ConstPtr &msg       = msg_evt.getConstMessage();
	std::string                      authority = msg_evt.getPublisherName();

	if (authority == "") {
		logger->log_warn(name(), "Message received without callerid");
	} else if (authority != ros::this_node::getName()) {
		tf_msg_queues_[active_queue_].push(msg);
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
RosTfThread::tf_message_cb(const ros::MessageEvent<tf2_msgs::TFMessage const> &msg_evt,
                           bool                                                static_tf)
{
	MutexLocker lock(tf_msg_queue_mutex_);

	const tf2_msgs::TFMessage::ConstPtr &msg       = msg_evt.getConstMessage();
	std::string                          authority = msg_evt.getPublisherName();

	if (authority == "") {
		logger->log_warn(name(), "Message received without callerid");
	} else if (authority != ros::this_node::getName()) {
		tf2_msg_queues_[active_queue_].push(std::make_pair(static_tf, msg));
	}
}
#endif
