
/***************************************************************************
 *  laserscan_thread.cpp - Thread to exchange laser scans
 *
 *  Created: Tue May 29 19:41:18 2012
 *  Copyright  2011-2012  Tim Niemueller [www.niemueller.de]
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

#include "laserscan_thread.h"

#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>

#include <fnmatch.h>

using namespace fawkes;

/** @class ROS2LaserScanThread "pcl_thread.h"
 * Thread to exchange point clouds between Fawkes and ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2LaserScanThread::ROS2LaserScanThread()
: Thread("ROS2LaserScanThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
  BlackBoardInterfaceListener("ROS2LaserScanThread")
{
	ls_msg_queue_mutex_ = new Mutex();
	seq_num_mutex_      = new Mutex();
}

/** Destructor. */
ROS2LaserScanThread::~ROS2LaserScanThread()
{
	delete ls_msg_queue_mutex_;
	delete seq_num_mutex_;
}

std::string
ROS2LaserScanThread::topic_name(const char *if_id, const char *suffix)
{
	std::string topic_name = std::string("fawkes_scans/") + if_id + "_" + suffix;
	std::string::size_type pos = 0;
	while ((pos = topic_name.find("-", pos)) != std::string::npos) {
		topic_name.replace(pos, 1, "_");
	}
	pos = 0;
	while ((pos = topic_name.find(" ", pos)) != std::string::npos) {
		topic_name.replace(pos, 1, "_");
	}
	return topic_name;
}

void
ROS2LaserScanThread::init()
{
	active_queue_ = 0;
	seq_num_      = 0;

	// Must do that before registering listener because we might already
	// get events right away
	rclcpp::SubscriptionOptionsBase subopts;
	subopts.ignore_local_publications = true;


	rclcpp::SubscriptionOptions options;
	options.ignore_local_publications = true;

	auto callback =
		[this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg,
			const rclcpp::MessageInfo & msg_info) -> void
		{
			MutexLocker lock(ls_msg_queue_mutex_);
			ls_msg_queues_[active_queue_].push({msg, msg_info});
		};


        cfg_tf_prefix_ = config->get_string_or_default("/ros2/tf/tf_prefix", "");
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	sub_ls_ = node_handle->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", qos, callback, options);

	ls360_ifs_  = blackboard->open_multiple_for_reading<Laser360Interface>("*");
	ls720_ifs_  = blackboard->open_multiple_for_reading<Laser720Interface>("*");
	ls1080_ifs_ = blackboard->open_multiple_for_reading<Laser1080Interface>("*");

	std::list<Laser360Interface *>::iterator i360;

	for (i360 = ls360_ifs_.begin(); i360 != ls360_ifs_.end(); ++i360) {
		(*i360)->read();
		logger->log_info(name(), "Opened %s", (*i360)->uid());
		bbil_add_data_interface(*i360);
		bbil_add_reader_interface(*i360);
		bbil_add_writer_interface(*i360);

		std::string topname = topic_name((*i360)->id(), "360");

		PublisherInfo pi;
		pi.pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(topname, 1);

		logger->log_info(name(), "Publishing laser scan %s at %s", (*i360)->uid(), topname.c_str());

		pi.msg.header.frame_id = cfg_tf_prefix_ + (*i360)->frame();
		pi.msg.angle_min       = 0;
		pi.msg.angle_max       = 2 * M_PI;
		pi.msg.angle_increment = deg2rad(1);
		pi.msg.ranges.resize(360);

		pubs_[(*i360)->uid()] = pi;
	}

	std::list<Laser720Interface *>::iterator i720;
	for (i720 = ls720_ifs_.begin(); i720 != ls720_ifs_.end(); ++i720) {
		logger->log_info(name(), "Opened %s", (*i720)->uid());
		bbil_add_data_interface(*i720);
		bbil_add_reader_interface(*i720);
		bbil_add_writer_interface(*i720);

		std::string topname = topic_name((*i720)->id(), "720");

		PublisherInfo pi;
		pi.pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(topname, 1);

		logger->log_info(name(), "Publishing laser scan %s at %s", (*i720)->uid(), topname.c_str());

		pi.msg.header.frame_id = cfg_tf_prefix_ + (*i720)->frame();
		pi.msg.angle_min       = 0;
		pi.msg.angle_max       = 2 * M_PI;
		pi.msg.angle_increment = deg2rad(0.5);
		pi.msg.ranges.resize(720);

		pubs_[(*i720)->uid()] = pi;
	}

	std::list<Laser1080Interface *>::iterator i1080;
	for (i1080 = ls1080_ifs_.begin(); i1080 != ls1080_ifs_.end(); ++i1080) {
		logger->log_info(name(), "Opened %s", (*i1080)->uid());
		bbil_add_data_interface(*i1080);
		bbil_add_reader_interface(*i1080);
		bbil_add_writer_interface(*i1080);

		std::string topname = topic_name((*i1080)->id(), "1080");

		PublisherInfo pi;
		pi.pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(topname, 1);

		logger->log_info(name(),
		                 "Publishing laser scan %s at %s, frame %s",
		                 (*i1080)->uid(),
		                 topname.c_str(),
		                 (*i1080)->frame());

		pi.msg.header.frame_id = cfg_tf_prefix_ + (*i1080)->frame();
		pi.msg.angle_min       = 0;
		pi.msg.angle_max       = 2 * M_PI;
		pi.msg.angle_increment = deg2rad(1. / 3.);
		pi.msg.ranges.resize(1080);

		pubs_[(*i1080)->uid()] = pi;
	}

	blackboard->register_listener(this);

	bbio_add_observed_create("Laser360Interface", "*");
	bbio_add_observed_create("Laser720Interface", "*");
	bbio_add_observed_create("Laser1080Interface", "*");
	blackboard->register_observer(this);
}

void
ROS2LaserScanThread::finalize()
{
	blackboard->unregister_listener(this);
	blackboard->unregister_observer(this);

	std::map<std::string, PublisherInfo>::iterator p;
	for (p = pubs_.begin(); p != pubs_.end(); ++p) {
	}

	std::list<Laser360Interface *>::iterator i360;
	for (i360 = ls360_ifs_.begin(); i360 != ls360_ifs_.end(); ++i360) {
		blackboard->close(*i360);
	}
	ls360_ifs_.clear();
	std::list<Laser720Interface *>::iterator i720;
	for (i720 = ls720_ifs_.begin(); i720 != ls720_ifs_.end(); ++i720) {
		blackboard->close(*i720);
	}
	ls720_ifs_.clear();
	std::list<Laser1080Interface *>::iterator i1080;
	for (i1080 = ls1080_ifs_.begin(); i1080 != ls1080_ifs_.end(); ++i1080) {
		blackboard->close(*i1080);
	}
	ls1080_ifs_.clear();
}

void
ROS2LaserScanThread::loop()
{
	ls_msg_queue_mutex_->lock();
	unsigned int queue = active_queue_;
	active_queue_      = 1 - active_queue_;
	ls_msg_queue_mutex_->unlock();

	while (!ls_msg_queues_[queue].empty()) {
		std::pair<std::shared_ptr<const sensor_msgs::msg::LaserScan>, const rclcpp::MessageInfo> tmp = ls_msg_queues_[queue].front();

		std::shared_ptr<const sensor_msgs::msg::LaserScan> msg = tmp.first;

		const rclcpp::MessageInfo minfo = tmp.second;
		// Check if interface exists, open if it does not
		const std::string callerid = minfo.get_rmw_message_info().publisher_gid.implementation_identifier;
		bool have_interface = true;
		if (ls360_wifs_.find(callerid) == ls360_wifs_.end()) {
			try {
				std::string        id      = std::string("ROS Laser ") + callerid;
				Laser360Interface *ls360if = blackboard->open_for_writing<Laser360Interface>(id.c_str());
				ls360_wifs_[callerid]      = ls360if;
			} catch (Exception &e) {
				logger->log_warn(name(),
									"Failed to open ROS laser interface for "
									"message from node %s, exception follows",
									callerid.c_str());
				logger->log_warn(name(), e);
				have_interface = false;
			}
		}

		if (have_interface) {
			// update interface with laser data
			Laser360Interface *ls360if = ls360_wifs_[callerid];
			ls360if->set_frame(msg->header.frame_id.c_str());
			float distances[360];
			for (unsigned int a = 0; a < 360; ++a) {
				float a_rad = deg2rad(a);
				if ((a_rad < msg->angle_min) || (a_rad > msg->angle_max)) {
					distances[a] = 0.;
				} else {
					// get closest ray from message
					int idx      = (int)roundf((a_rad - msg->angle_min) / msg->angle_increment);
					distances[a] = msg->ranges[idx];
				}
			}
			ls360if->set_distances(distances);
			ls360if->write();
		}
		ls_msg_queues_[queue].pop();
	}
}

void
ROS2LaserScanThread::bb_interface_data_refreshed(fawkes::Interface *interface) throw()
{
	Laser360Interface * ls360if  = dynamic_cast<Laser360Interface *>(interface);
	Laser720Interface * ls720if  = dynamic_cast<Laser720Interface *>(interface);
	Laser1080Interface *ls1080if = dynamic_cast<Laser1080Interface *>(interface);

	PublisherInfo &         pi  = pubs_[interface->uid()];
	sensor_msgs::msg::LaserScan &msg = pi.msg;

	if (ls360if) {
		ls360if->read();

		const Time *time = ls360if->timestamp();

		seq_num_mutex_->lock();
		//msg.header.seq = ++seq_num_;
		seq_num_mutex_->unlock();
		msg.header.stamp    = rclcpp::Time(time->get_sec(), time->get_nsec());
		msg.header.frame_id = cfg_tf_prefix_ + ls360if->frame();

		msg.angle_min       = 0;
		msg.angle_max       = 2 * M_PI;
		msg.angle_increment = deg2rad(1);
		msg.range_min       = 0.;
		msg.range_max       = 1000.;
		msg.ranges.resize(360);
		memcpy(&msg.ranges[0], ls360if->distances(), 360 * sizeof(float));

		pi.pub->publish(pi.msg);

	} else if (ls720if) {
		ls720if->read();

		const Time *time = ls720if->timestamp();

		seq_num_mutex_->lock();
		//msg.header.seq = ++seq_num_;
		seq_num_mutex_->unlock();
		msg.header.stamp    = rclcpp::Time(time->get_sec(), time->get_nsec());
		msg.header.frame_id = cfg_tf_prefix_ + ls720if->frame();

		msg.angle_min       = 0;
		msg.angle_max       = 2 * M_PI;
		msg.angle_increment = deg2rad(1. / 2.);
		msg.range_min       = 0.;
		msg.range_max       = 1000.;
		msg.ranges.resize(720);
		memcpy(&msg.ranges[0], ls720if->distances(), 720 * sizeof(float));

		pi.pub->publish(pi.msg);

	} else if (ls1080if) {
		ls1080if->read();

		const Time *time = ls1080if->timestamp();

		seq_num_mutex_->lock();
		//msg.header.seq = ++seq_num_;
		seq_num_mutex_->unlock();
		msg.header.stamp    = rclcpp::Time(time->get_sec(), time->get_nsec());
		msg.header.frame_id = cfg_tf_prefix_ + ls1080if->frame();

		msg.angle_min       = 0;
		msg.angle_max       = 2 * M_PI;
		msg.angle_increment = deg2rad(1. / 3.);
		msg.range_min       = 0.;
		msg.range_max       = 1000.;
		msg.ranges.resize(1080);
		memcpy(&msg.ranges[0], ls1080if->distances(), 1080 * sizeof(float));

		pi.pub->publish(pi.msg);
	}
}

void
ROS2LaserScanThread::bb_interface_created(const char *type, const char *id) throw()
{
	// Ignore ID pattern of our own interfaces
	if (fnmatch("ROS *", id, FNM_NOESCAPE) == 0)
		return;

	if (strncmp(type, "Laser360Interface", INTERFACE_TYPE_SIZE_) == 0) {
		Laser360Interface *ls360if;
		try {
			logger->log_info(name(), "Opening %s:%s", type, id);
			ls360if = blackboard->open_for_reading<Laser360Interface>(id);
		} catch (Exception &e) {
			// ignored
			logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
			return;
		}

		try {
			bbil_add_data_interface(ls360if);
			bbil_add_reader_interface(ls360if);
			bbil_add_writer_interface(ls360if);

			std::string topname = topic_name(ls360if->id(), "360");

			PublisherInfo pi;
			pi.pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(topname, 1);

			logger->log_info(name(), "Publishing laser scan %s at %s", ls360if->uid(), topname.c_str());

			pi.msg.header.frame_id = cfg_tf_prefix_ + ls360if->frame();
			pi.msg.angle_min       = 0;
			pi.msg.angle_max       = 2 * M_PI;
			pi.msg.angle_increment = deg2rad(1);
			pi.msg.ranges.resize(360);

			pubs_[ls360if->uid()] = pi;

			blackboard->update_listener(this);
			ls360_ifs_.push_back(ls360if);
		} catch (Exception &e) {
			blackboard->close(ls360if);
			logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
			return;
		}

	} else if (strncmp(type, "Laser720Interface", INTERFACE_TYPE_SIZE_) == 0) {
		Laser720Interface *ls720if;
		try {
			logger->log_info(name(), "Opening %s:%s", type, id);
			ls720if = blackboard->open_for_reading<Laser720Interface>(id);
		} catch (Exception &e) {
			// ignored
			logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
			return;
		}

		try {
			bbil_add_data_interface(ls720if);
			bbil_add_reader_interface(ls720if);
			bbil_add_writer_interface(ls720if);

			std::string topname = topic_name(ls720if->id(), "720");

			PublisherInfo pi;
			pi.pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(topname, 1);

			logger->log_info(name(), "Publishing laser scan %s at %s", ls720if->uid(), topname.c_str());

			pi.msg.header.frame_id = cfg_tf_prefix_ + ls720if->frame();
			pi.msg.angle_min       = 0;
			pi.msg.angle_max       = 2 * M_PI;
			pi.msg.angle_increment = deg2rad(0.5);
			pi.msg.ranges.resize(720);

			pubs_[ls720if->uid()] = pi;

			blackboard->update_listener(this);
			ls720_ifs_.push_back(ls720if);
		} catch (Exception &e) {
			blackboard->close(ls720if);
			logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
			return;
		}

	} else if (strncmp(type, "Laser1080Interface", INTERFACE_TYPE_SIZE_) == 0) {
		Laser1080Interface *ls1080if;
		try {
			logger->log_info(name(), "Opening %s:%s", type, id);
			ls1080if = blackboard->open_for_reading<Laser1080Interface>(id);
		} catch (Exception &e) {
			// ignored
			logger->log_warn(name(), "Failed to open %s:%s: %s", type, id, e.what());
			return;
		}

		try {
			bbil_add_data_interface(ls1080if);
			bbil_add_reader_interface(ls1080if);
			bbil_add_writer_interface(ls1080if);

			std::string topname = topic_name(ls1080if->id(), "1080");

			PublisherInfo pi;
			pi.pub = node_handle->create_publisher<sensor_msgs::msg::LaserScan>(topname, 1);

			logger->log_info(name(),
			                 "Publishing 1080 laser scan %s at %s",
			                 ls1080if->uid(),
			                 topname.c_str());

			pi.msg.header.frame_id = cfg_tf_prefix_ + ls1080if->frame();
			pi.msg.angle_min       = 0;
			pi.msg.angle_max       = 2 * M_PI;
			pi.msg.angle_increment = deg2rad(0.5);
			pi.msg.ranges.resize(1080);

			pubs_[ls1080if->uid()] = pi;

			blackboard->update_listener(this);
			ls1080_ifs_.push_back(ls1080if);
		} catch (Exception &e) {
			blackboard->close(ls1080if);
			logger->log_warn(name(), "Failed to register for %s:%s: %s", type, id, e.what());
			return;
		}
	}
}

void
ROS2LaserScanThread::bb_interface_writer_removed(fawkes::Interface *interface,
                                                unsigned int       instance_serial) throw()
{
	conditional_close(interface);
}

void
ROS2LaserScanThread::bb_interface_reader_removed(fawkes::Interface *interface,
                                                unsigned int       instance_serial) throw()
{
	conditional_close(interface);
}

void
ROS2LaserScanThread::conditional_close(Interface *interface) throw()
{
	// Verify it's a laser interface
	Laser360Interface * ls360if  = dynamic_cast<Laser360Interface *>(interface);
	Laser720Interface * ls720if  = dynamic_cast<Laser720Interface *>(interface);
	Laser1080Interface *ls1080if = dynamic_cast<Laser1080Interface *>(interface);

	if (ls360if) {
		std::list<Laser360Interface *>::iterator i;
		for (i = ls360_ifs_.begin(); i != ls360_ifs_.end(); ++i) {
			if (*ls360if == **i) {
				if (!ls360if->has_writer() && (ls360if->num_readers() == 1)) {
					// It's only us
					logger->log_info(name(), "Last on %s, closing", ls360if->uid());
					bbil_remove_data_interface(*i);
					bbil_remove_reader_interface(*i);
					bbil_remove_writer_interface(*i);
					blackboard->update_listener(this);
					blackboard->close(*i);
					ls360_ifs_.erase(i);
					break;
				}
			}
		}
	} else if (ls720if) {
		std::list<Laser720Interface *>::iterator i;
		for (i = ls720_ifs_.begin(); i != ls720_ifs_.end(); ++i) {
			if (*ls720if == **i) {
				if (!ls720if->has_writer() && (ls720if->num_readers() == 1)) {
					// It's only us
					logger->log_info(name(), "Last on %s, closing", ls720if->uid());
					bbil_remove_data_interface(*i);
					bbil_remove_reader_interface(*i);
					bbil_remove_writer_interface(*i);
					blackboard->update_listener(this);
					blackboard->close(*i);
					ls720_ifs_.erase(i);
					break;
				}
			}
		}

	} else if (ls1080if) {
		std::list<Laser1080Interface *>::iterator i;
		for (i = ls1080_ifs_.begin(); i != ls1080_ifs_.end(); ++i) {
			if (*ls1080if == **i) {
				if (!ls1080if->has_writer() && (ls1080if->num_readers() == 1)) {
					// It's only us
					logger->log_info(name(), "Last on %s, closing", ls1080if->uid());
					bbil_remove_data_interface(*i);
					bbil_remove_reader_interface(*i);
					bbil_remove_writer_interface(*i);
					blackboard->update_listener(this);
					blackboard->close(*i);
					ls1080_ifs_.erase(i);
					break;
				}
			}
		}
	}
}

/** Callback function for ROS laser scan message subscription.
 * @param msg incoming message
 * std::shared_ptr<const sensor_msgs::msg::LaserScan>, const rclcpp::MessageInfo
 */
void
ROS2LaserScanThread::laser_scan_message_cb(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg, const rclcpp::MessageInfo &msg_info)
{
	MutexLocker lock(ls_msg_queue_mutex_);
	ls_msg_queues_[active_queue_].push({msg, msg_info});
}
