
/***************************************************************************
 *  laserscan_thread.h - Thread to exchange laser scans
 *
 *  Created: Tue May 29 19:32:39 2012
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

#ifndef _PLUGINS_ROS_LASERSCAN_THREAD_H_
#define _PLUGINS_ROS_LASERSCAN_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <interfaces/Laser1080Interface.h>
#include <interfaces/Laser360Interface.h>
#include <interfaces/Laser720Interface.h>
#include <plugins/ros2/aspect/ros2.h>
#include <utils/time/time.h>

#include <list>
#include <queue>
#include <memory>
using std::placeholders::_1;

//ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class ROS2LaserScanThread : public fawkes::Thread,
                           public fawkes::LoggingAspect,
                           public fawkes::ConfigurableAspect,
                           public fawkes::BlockedTimingAspect,
                           public fawkes::BlackBoardAspect,
                           public fawkes::ROS2Aspect,
                           public fawkes::BlackBoardInterfaceObserver,
                           public fawkes::BlackBoardInterfaceListener
{
public:
	ROS2LaserScanThread();
	virtual ~ROS2LaserScanThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	// for BlackBoardInterfaceObserver
	virtual void bb_interface_created(const char *type, const char *id) throw();

	// for BlackBoardInterfaceListener
	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) throw();
	virtual void bb_interface_writer_removed(fawkes::Interface *interface,
	                                         unsigned int       instance_serial) throw();
	virtual void bb_interface_reader_removed(fawkes::Interface *interface,
	                                         unsigned int       instance_serial) throw();

private:
	void        laser_scan_message_cb(std::shared_ptr<const sensor_msgs::msg::LaserScan> msg, const rclcpp::MessageInfo &msg_info);
	void        conditional_close(fawkes::Interface *interface) throw();
	std::string topic_name(const char *if_id, const char *suffix);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	std::list<fawkes::Laser360Interface *>  ls360_ifs_;
	std::list<fawkes::Laser720Interface *>  ls720_ifs_;
	std::list<fawkes::Laser1080Interface *> ls1080_ifs_;

	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_ls_;

	/// @cond INTERNALS
	typedef struct
	{
		rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
		sensor_msgs::msg::LaserScan msg;
	} PublisherInfo;
	/// @endcond
	std::map<std::string, PublisherInfo> pubs_;

        std::string     cfg_tf_prefix_;

	fawkes::Mutex *                                             ls_msg_queue_mutex_;
	unsigned int                                                active_queue_;
	std::queue<std::pair<std::shared_ptr<const sensor_msgs::msg::LaserScan>, const rclcpp::MessageInfo> > ls_msg_queues_[2];

	std::map<std::string, fawkes::Laser360Interface *> ls360_wifs_;

	fawkes::Mutex *seq_num_mutex_;
	unsigned int   seq_num_;

};

#endif
