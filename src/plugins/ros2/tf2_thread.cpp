
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

#include "tf2_thread.h"

#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class ROS2TF2Thread "tf_thread.h"
 * Thread to exchange transforms between Fawkes and ROS.
 * This threads connects to Fawkes and ROS to read and write transforms.
 * Transforms received on one end are republished to the other side. To
 * Fawkes new frames are published during the sensor hook.
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2TF2Thread::ROS2TF2Thread()
: Thread("ROS2TF2Thread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
  TransformAspect(TransformAspect::DEFER_PUBLISHER),
  BlackBoardInterfaceListener("ROS2TF2Thread")
{
	tf_msg_queue_mutex_ = new Mutex();
	seq_num_mutex_      = new Mutex();
	last_update_        = new Time();
}

/** Destructor. */
ROS2TF2Thread::~ROS2TF2Thread()
{
	delete tf_msg_queue_mutex_;
	delete seq_num_mutex_;
	delete last_update_;
}

void
ROS2TF2Thread::init()
{
	active_queue_ = 0;
	seq_num_      = 0;
	last_update_->set_clock(clock);
	last_update_->set_time(0, 0);

	cfg_update_interval_ = config->get_float_or_default("/ros2/tf/static-update-interval", 1.0);
	cfg_tf_prefix_ = config->get_string_or_default("/ros2/tf/tf_prefix", "");
	cfg_tf_prefix_exclusions_ = config->get_strings_or_defaults("/ros2/tf/tf_prefix_exclusions", std::vector<std::string>());
	cfg_use_namespace_ = config->get_bool_or_default("/ros2/tf/use_namespace", false);
	tf_prefix_enabled_ = false;

        if (cfg_tf_prefix_ == "$HOSTNAME") {
                HostInfo hinfo;
                // namespace must not contain characters other than alphanumerics, '_', or '/'
                cfg_tf_prefix_ = hinfo.short_name();
                std::regex tf_prefix_pattern("[^A-Za-z0-9_]");

                // write the results to an output iterator
                cfg_tf_prefix_ = std::regex_replace(cfg_tf_prefix_,
                                                    tf_prefix_pattern, "") + std::string("_");
        }
        if (cfg_tf_prefix_ != "") {
		tf_prefix_enabled_ = true;
        }

//	tf_prefix_ = cfg_tf_prefix_;
//        std::cout << "set prefix: " << tf_prefix_.c_str() << std::endl;
	// Must do that before registering listener because we might already
	// get events right away
	rclcpp::SubscriptionOptionsBase subopts;
	subopts.ignore_local_publications = true;

        std::string tf_topic = "tf";
        std::string tf_static_topic = "tf_static";

	if (cfg_use_namespace_ == false) {
		tf_topic.insert(0, 1, '/');
		tf_static_topic.insert(0, 1, '/');
	}

	sub_tf_        = node_handle->create_subscription<tf2_msgs::msg::TFMessage>(tf_topic, 1, std::bind(&ROS2TF2Thread::tf_message_cb_dynamic, this, _1), rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>(subopts));
	sub_static_tf_ = node_handle->create_subscription<tf2_msgs::msg::TFMessage>(tf_static_topic, rclcpp::QoS(1).transient_local(), std::bind(&ROS2TF2Thread::tf_message_cb_static, this, _1), rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>(subopts));
	pub_tf_        = node_handle->create_publisher<tf2_msgs::msg::TFMessage>(tf_topic, 1);
	pub_static_tf_ = node_handle->create_publisher<tf2_msgs::msg::TFMessage>(tf_static_topic, rclcpp::QoS(1).transient_local());

	tfifs_ = blackboard->open_multiple_for_reading<TransformInterface>("/tf*");
	std::list<TransformInterface *>::iterator i;
	for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
		bbil_add_data_interface(*i);
		bbil_add_reader_interface(*i);
		bbil_add_writer_interface(*i);
	}
	blackboard->register_listener(this);

	publish_static_transforms_to_ros2();

	bbio_add_observed_create("TransformInterface", "/tf*");
	blackboard->register_observer(this);
}

void
ROS2TF2Thread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->unregister_observer(this);

	rclcpp::shutdown();

	std::list<TransformInterface *>::iterator i;
	for (i = tfifs_.begin(); i != tfifs_.end(); ++i) {
		blackboard->close(*i);
	}
	tfifs_.clear();
}

void
ROS2TF2Thread::loop()
{
	tf_msg_queue_mutex_->lock();
	unsigned int queue = active_queue_;
	active_queue_      = 1 - active_queue_;
	tf_msg_queue_mutex_->unlock();
	while (!tf2_msg_queues_[queue].empty()) {
		const std::pair<bool, tf2_msgs::msg::TFMessage::ConstPtr> &q     = tf2_msg_queues_[queue].front();
		const tf2_msgs::msg::TFMessage::ConstPtr &                 msg   = q.second;
		const size_t                                          tsize = msg->transforms.size();
		for (size_t i = 0; i < tsize; ++i) {
			publish_transform_to_fawkes(msg->transforms[i], q.first);
		}
		tf2_msg_queues_[queue].pop();
	}

	fawkes::Time now(clock);
	if ((now - last_update_) > cfg_update_interval_) {
		last_update_->stamp();

		publish_static_transforms_to_ros2();
	}
}

void
ROS2TF2Thread::bb_interface_data_refreshed(fawkes::Interface *interface) throw()
{
	TransformInterface *tfif = dynamic_cast<TransformInterface *>(interface);
	if (!tfif)
		return;

	tfif->read();

	if (cfg_use_tf2_ && tfif->is_static_transform()) {
		publish_static_transforms_to_ros2();
	} else {
		tf2_msgs::msg::TFMessage tmsg;
		tmsg.transforms.push_back(create_transform_stamped(tfif));
		pub_tf_->publish(tmsg);
	}
}

void
ROS2TF2Thread::bb_interface_created(const char *type, const char *id) throw()
{
	if (strncmp(type, "TransformInterface", INTERFACE_TYPE_SIZE_) != 0)
		return;

	for (const auto &f : ros2_frames_) {
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
ROS2TF2Thread::bb_interface_writer_removed(fawkes::Interface *interface,
                                         unsigned int       instance_serial) throw()
{
	conditional_close(interface);
}

void
ROS2TF2Thread::bb_interface_reader_removed(fawkes::Interface *interface,
                                         unsigned int       instance_serial) throw()
{
	conditional_close(interface);
}

void
ROS2TF2Thread::conditional_close(Interface *interface) throw()
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

geometry_msgs::msg::TransformStamped
ROS2TF2Thread::create_transform_stamped(TransformInterface *tfif, const Time *time)
{
	double *translation = tfif->translation();
	double *rotation    = tfif->rotation();
	if (!time)
		time = tfif->timestamp();

	geometry_msgs::msg::Vector3 t;
	t.x = translation[0];
	t.y = translation[1];
	t.z = translation[2];
	geometry_msgs::msg::Quaternion r;
	r.x = rotation[0];
	r.y = rotation[1];
	r.z = rotation[2];
	r.w = rotation[3];
	geometry_msgs::msg::Transform tr;
	tr.translation = t;
	tr.rotation    = r;

	geometry_msgs::msg::TransformStamped ts;
	ts.header.stamp    = rclcpp::Time(time->get_sec(), time->get_nsec());
	ts.header.frame_id = tfif->frame();
	ts.child_frame_id  = tfif->child_frame();

	if (tf_prefix_enabled_ == true) {
		if (std::find(cfg_tf_prefix_exclusions_.begin(), cfg_tf_prefix_exclusions_.end(), ts.header.frame_id) == cfg_tf_prefix_exclusions_.end()) {
		// cfg_tf_prefix_exclusions_ does not contain ts.header.frame_id so we can add the prefix.
			ts.header.frame_id.insert(0, cfg_tf_prefix_);
		}
		if (std::find(cfg_tf_prefix_exclusions_.begin(), cfg_tf_prefix_exclusions_.end(), ts.child_frame_id) == cfg_tf_prefix_exclusions_.end()) {
		// cfg_tf_prefix_exclusions_ does not contain ts.header.frame_id so we can add the prefix.
			ts.child_frame_id.insert(0, cfg_tf_prefix_);
		}
	}
	ts.transform       = tr;

	return ts;
}

void
ROS2TF2Thread::publish_static_transforms_to_ros2()
{
	std::list<fawkes::TransformInterface *>::iterator t;
	fawkes::Time                                      now(clock);

	tf2_msgs::msg::TFMessage tmsg;
	for (t = tfifs_.begin(); t != tfifs_.end(); ++t) {
		fawkes::TransformInterface *tfif = *t;
		tfif->read();
		if (tfif->is_static_transform()) {
			tmsg.transforms.push_back(create_transform_stamped(tfif, &now));
		}
	}
	pub_static_tf_->publish(tmsg);
}

void
ROS2TF2Thread::publish_transform_to_fawkes(const geometry_msgs::msg::TransformStamped &ts, bool static_tf)
{
	const geometry_msgs::msg::Vector3 &   t = ts.transform.translation;
	const geometry_msgs::msg::Quaternion &r = ts.transform.rotation;

	std::string frame_id = ts.header.frame_id;
	std::string child_frame_id = ts.child_frame_id;

	// We need to ignore transforms that were published with the tf_prefix prepended by this instance.
	// I.e. if Fawkes maintains a transform map -> odom and adds the prefix map -> robot1_odom
	// the latter should not be considered as an additional transform to be published to Fawkes.
	// If Fawkes publishes this particular transform as map -> robot1_odom it would recognize this as a
	// transform available on the ROS side and hence consider it as a new transform for Fawkes.
	if (tf_prefix_enabled_ == true) {
		frame_id = std::regex_replace(frame_id, std::regex(cfg_tf_prefix_), "");
		child_frame_id = std::regex_replace(child_frame_id, std::regex(cfg_tf_prefix_), "");
	}


	fawkes::Time time(ts.header.stamp.sec, ts.header.stamp.nanosec / 1000);

	fawkes::tf::Transform        tr(fawkes::tf::Quaternion(r.x, r.y, r.z, r.w),
                           fawkes::tf::Vector3(t.x, t.y, t.z));
	fawkes::tf::StampedTransform st(tr, time, frame_id, child_frame_id);

	if (tf_publishers.find(child_frame_id) == tf_publishers.end()) {
		try {
			ros2_frames_.push_back(std::string("/tf/") + child_frame_id);
			tf_add_publisher("%s", child_frame_id.c_str());
			tf_publishers[ts.child_frame_id]->send_transform(st, static_tf);
		} catch (Exception &e) {
			ros2_frames_.pop_back();
			logger->log_warn(name(),
			                 "Failed to create Fawkes transform publisher for frame %s from ROS",
			                 child_frame_id.c_str());
			logger->log_warn(name(), e);
		}
	} else {
		tf_publishers[child_frame_id]->send_transform(st, static_tf);
	}
}

void
ROS2TF2Thread::tf_message_cb_static(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
	tf_message_cb(msg, true);
}

void
ROS2TF2Thread::tf_message_cb_dynamic(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
	tf_message_cb(msg, false);
}

void
ROS2TF2Thread::tf_message_cb(const tf2_msgs::msg::TFMessage::SharedPtr tf2_message, bool stat)
{
	MutexLocker lock(tf_msg_queue_mutex_);
	tf2_msg_queues_[active_queue_].push(std::make_pair(stat, tf2_message));
}
