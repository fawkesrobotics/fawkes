
/***************************************************************************
 *  pcl_thread.cpp - Thread to exchange point clouds
 *
 *  Created: Mon Nov 07 02:26:35 2011
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

#ifndef _PLUGINS_ROS_PCL_THREAD_H_
#define _PLUGINS_ROS_PCL_THREAD_H_

#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/pointcloud.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <core/threading/mutex.h>
#include <core/threading/thread.h>
#include <interfaces/TransformInterface.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_utils/pcl_adapter.h>
#include <plugins/ros/aspect/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <utils/time/time.h>

#include <list>
#include <queue>

class RosPointCloudThread : public fawkes::Thread,
                            public fawkes::ClockAspect,
                            public fawkes::LoggingAspect,
                            public fawkes::ConfigurableAspect,
                            public fawkes::BlockedTimingAspect,
                            public fawkes::PointCloudAspect,
                            public fawkes::ROSAspect
{
public:
	RosPointCloudThread();
	virtual ~RosPointCloudThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private:
	void ros_pointcloud_search();
	void ros_pointcloud_check_for_listener_in_fawkes();
	void fawkes_pointcloud_publish_to_ros();
	void fawkes_pointcloud_search();
	void ros_pointcloud_on_data_msg(const sensor_msgs::PointCloud2ConstPtr &msg,
	                                const std::string                      &topic_name);

	template <typename PointT>
	void
	add_pointcloud(const sensor_msgs::PointCloud2ConstPtr &msg, const std::string topic_name)
	{
		fawkes::RefPtr<pcl::PointCloud<PointT>> pcl;
		pcl = new pcl::PointCloud<PointT>();
		pcl::fromROSMsg(*msg, **pcl);
		pcl_manager->add_pointcloud(topic_name.c_str(), pcl);
		ros_pointcloud_available_ref_[topic_name] =
		  new fawkes::pcl_utils::PointCloudStorageAdapter<PointT>(pcl);
	}

	template <typename PointT>
	void
	update_pointcloud(const sensor_msgs::PointCloud2ConstPtr &msg, const std::string topic_name)
	{
		fawkes::RefPtr<pcl::PointCloud<PointT>> pcl;
		pcl = dynamic_cast<fawkes::pcl_utils::PointCloudStorageAdapter<PointT> *>(
		        ros_pointcloud_available_ref_[topic_name])
		        ->cloud;
		pcl::fromROSMsg(*msg, **pcl);
	}

	PointCloudAdapter *adapter_;

	/// @cond INTERNALS
	typedef struct
	{
		ros::Publisher           pub;
		sensor_msgs::PointCloud2 msg;
		fawkes::Time             last_sent;
	} PublisherInfo;
	/// @endcond
	std::map<std::string, PublisherInfo> fawkes_pubs_; // the list and ref of topics from fawkes->ros
	std::list<std::string> ros_pointcloud_available_;  // the list of topics from ros->fawkes
	std::map<std::string, fawkes::pcl_utils::StorageAdapter *>
	  ros_pointcloud_available_ref_; // the list of refs of topics from ros->fawkes
	std::map<std::string, ros::Subscriber>
	  ros_pointcloud_subs_; // the list of subscribers in ros, ros_pointcloud_available that are currently used in fawkes

	fawkes::Time ros_pointcloud_last_searched_;

	float cfg_ros_research_ival_;
};

#endif
