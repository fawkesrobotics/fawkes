
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

#ifndef _PLUGINS_ROS2_TF2_THREAD_H_
#define _PLUGINS_ROS2_TF2_THREAD_H_

#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <interfaces/TransformInterface.h>
#include <plugins/ros2/aspect/ros2.h>

#include <list>
#include <queue>
#include <memory>
using std::placeholders::_1;

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>

class ROS2TF2Thread : public fawkes::Thread,
                    public fawkes::ClockAspect,
                    public fawkes::LoggingAspect,
                    public fawkes::ConfigurableAspect,
                    public fawkes::BlockedTimingAspect,
                    public fawkes::BlackBoardAspect,
                    public fawkes::TransformAspect,
                    public fawkes::ROS2Aspect,
                    public fawkes::BlackBoardInterfaceObserver,
                    public fawkes::BlackBoardInterfaceListener
{
public:
	ROS2TF2Thread();
	virtual ~ROS2TF2Thread();

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
	void tf_message_cb_dynamic(const tf2_msgs::msg::TFMessage::SharedPtr msg);
	void tf_message_cb_static(const tf2_msgs::msg::TFMessage::SharedPtr msg);
	void tf_message_cb(const tf2_msgs::msg::TFMessage::SharedPtr msg, bool stat);

	void conditional_close(fawkes::Interface *interface) throw();
	void publish_static_transforms_to_ros2();
	void publish_transform_to_fawkes(const geometry_msgs::msg::TransformStamped &ts,
	                                 bool                                   static_tf = false);
	geometry_msgs::msg::TransformStamped create_transform_stamped(fawkes::TransformInterface *tfif,
	                                                         const fawkes::Time *        time = NULL);

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	bool		cfg_use_tf2_;
	bool		cfg_use_namespace_;
	std::string	cfg_tf_prefix_;
	std::vector<std::string>	cfg_tf_prefix_exclusions_;
	float		cfg_update_interval_;

	bool		tf_prefix_enabled_;

	std::list<std::string>                  ros2_frames_;
	std::list<fawkes::TransformInterface *> tfifs_;

	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
	rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_static_tf_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr  pub_tf_;
	rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr  pub_static_tf_;

	fawkes::Mutex *                       tf_msg_queue_mutex_;
	unsigned int                          active_queue_;
	std::queue<std::pair<bool, tf2_msgs::msg::TFMessage::ConstPtr>> tf2_msg_queues_[2];
	fawkes::Mutex *seq_num_mutex_;
	unsigned int   seq_num_;

	fawkes::Time *last_update_;
};

#endif
