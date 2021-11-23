
/***************************************************************************
 *  pcl_thread.cpp - Thread to exchange point clouds
 *
 *  Created: Mon Nov 07 02:58:40 2011
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

#include "pcl_thread.h"

#include <core/threading/mutex_locker.h>

using namespace fawkes;

/** @class ROS2PointCloudThread "pcl_thread.h"
 * Thread to exchange point clouds between Fawkes and ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
ROS2PointCloudThread::ROS2PointCloudThread()
: Thread("ROS2PointCloudThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

/** Destructor. */
ROS2PointCloudThread::~ROS2PointCloudThread()
{
}

void
ROS2PointCloudThread::init()
{
	//pubman_ = new RosPointCloudPublisherManager(rosnode);
	adapter_ = new PointCloudAdapter(pcl_manager, logger);

	cfg_ros_research_ival_ = config->get_float("/ros/pcl/ros-search-interval");

	fawkes_pointcloud_search();

	ros_pointcloud_search();
	ros_pointcloud_last_searched_.stamp();
}

void
ROS2PointCloudThread::finalize()
{
	for (const std::string &item : ros_pointcloud_available_) {
		pcl_manager->remove_pointcloud(item.c_str());
	}
	for (const std::pair<std::string, fawkes::pcl_utils::StorageAdapter *> &item :
	     ros_pointcloud_available_ref_) {
		delete item.second;
	}
	for (std::pair<std::string, ros::Subscriber> item : ros_pointcloud_subs_) {
		item.second.shutdown();
	}
	delete adapter_;
}

void
ROS2PointCloudThread::loop()
{
	// search ever n sec for new clouds on ROS side
	fawkes::Time now;
	now.stamp();
	if (fawkes::time_diff_sec(*now.get_timeval(), *ros_pointcloud_last_searched_.get_timeval())
	    >= cfg_ros_research_ival_) {
		ros_pointcloud_last_searched_ = now;
		ros_pointcloud_search();
		ros_pointcloud_check_for_listener_in_fawkes();
		//TODO if fawkes_pointcloud_search() would be called here, a check for clouds from fawkes need to be implemented
	}

	// publish clouds fawkes->ros
	fawkes_pointcloud_publish_to_ros();
	// publish clouds ros->fawkes (this is done in callbacks)
}

void
ROS2PointCloudThread::ros_pointcloud_search()
{
	std::list<std::string> ros_pointclouds_new;

	// get all ROS topics
	std::map<std::string, std::vector<std::string>> ros_topics;
	
	ros_topics = node_handle->get_topc_names_and_types();
	if (!ros_topics) {
		logger->log_info(name(), "Coulnd't get available ROS topics");
		return;
	}

	// iterate through all topics
	std::map<std::string, std::vector<std::string>>::iterator it;
	for (it = ros_topics.begin(); it != ros_topics.end(); it++) {
		// only topics of type sensor_msgs/PointCloud2 are important
		if (0 == it.second[0].compare("sensor_msgs/PointCloud2")) {
			// check if this is a topic comming from fawkes
			bool topic_not_from_fawkes = true;
			for (const std::pair<std::string, PublisherInfo> &fawkes_cloud : fawkes_pubs_) {
				if (0 == it.first.compare(fawkes_cloud.second.pub.getTopic())) {
					topic_not_from_fawkes = false;
				}
			}
			if (topic_not_from_fawkes) {
				ros_pointclouds_new.push_back(it.first);
			}
		}
	}

	// check for removed clouds
	std::list<std::string> items_to_remove;
	for (const std::string &item_old : ros_pointcloud_available_) {
		bool exists = false;
		for (std::string item_new : ros_pointclouds_new) {
			if (0 == item_old.compare(item_new)) {
				exists = true;
				break;
			}
		}
		if (!exists) {
			items_to_remove.push_back(item_old);
		}
	}
	for (const std::string &item : items_to_remove) {
		logger->log_info(name(), "Pointcloud %s is not available from ROS anymore", item.c_str());
		ros_pointcloud_available_.remove(item);
	}

	// check for new clouds
	for (const std::string &ros_topic : ros_pointclouds_new) {
		bool exists = false;
		for (const std::string &in_list : ros_pointcloud_available_) {
			if (0 == ros_topic.compare(in_list)) {
				exists = true;
				break;
			}
		}
		if (!exists) {
			logger->log_info(name(), "Pointcloud %s is now available from ROS", ros_topic.c_str());
			ros_pointcloud_available_.push_back(ros_topic);
			ros_pointcloud_subs_[ros_topic] = node_handle->create_subscription<sensor_msgs::msg::PointCloud2>(
			  ros_topic,
			  1,
			  boost::bind(&ROS2PointCloudThread::ros_pointcloud_on_data_msg, this, _1, ros_topic));
		}
	}
}

void
ROS2PointCloudThread::ros_pointcloud_check_for_listener_in_fawkes()
{
	for (const std::pair<std::string, fawkes::pcl_utils::StorageAdapter *> &item :
	     ros_pointcloud_available_ref_) {
		unsigned int use_count = 0;
		if (item.second->is_pointtype<pcl::PointXYZ>()) {
			use_count =
			  dynamic_cast<fawkes::pcl_utils::PointCloudStorageAdapter<pcl::PointXYZ> *>(item.second)
			    ->cloud.use_count();
		} else if (item.second->is_pointtype<pcl::PointXYZRGB>()) {
			use_count =
			  dynamic_cast<fawkes::pcl_utils::PointCloudStorageAdapter<pcl::PointXYZRGB> *>(item.second)
			    ->cloud.use_count();
		} else if (item.second->is_pointtype<pcl::PointXYZI>()) {
			use_count =
			  dynamic_cast<fawkes::pcl_utils::PointCloudStorageAdapter<pcl::PointXYZI> *>(item.second)
			    ->cloud.use_count();
		} else {
			logger->log_error(name(), "Can't detect cloud type");
		}

		if (
		  use_count
		  <= 2) { // my internal list, this ref and the pcl_manager have copys of this pointer, if more are used, otheres are listening too
			std::map<std::string, ros::Subscriber>::iterator element =
			  ros_pointcloud_subs_.find(item.first);
			if (element != ros_pointcloud_subs_.end()) {
				ros_pointcloud_subs_.erase(item.first);
			}
		} else {
			ros_pointcloud_subs_[item.first] = rosnode->subscribe<sensor_msgs::PointCloud2>(
			  item.first,
			  1,
			  boost::bind(&ROS2PointCloudThread::ros_pointcloud_on_data_msg, this, _1, item.first));
		}
	}
}

void
ROS2PointCloudThread::fawkes_pointcloud_search()
{
	std::vector<std::string> pcls = pcl_manager->get_pointcloud_list();

	std::vector<std::string>::iterator p;
	for (p = pcls.begin(); p != pcls.end(); ++p) {
		std::string            topic_name = std::string("fawkes_pcls/") + *p;
		std::string::size_type pos        = 0;
		while ((pos = topic_name.find("-", pos)) != std::string::npos) {
			topic_name.replace(pos, 1, "_");
		}

		PublisherInfo pi;
		pi.pub = node_handle->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 1);

		logger->log_info(name(), "Publishing point cloud %s at %s", p->c_str(), topic_name.c_str());

		std::string                         frame_id;
		unsigned int                        width, height;
		bool                                is_dense;
		PointCloudAdapter::V_PointFieldInfo fieldinfo;
		adapter_->get_info(*p, width, height, frame_id, is_dense, fieldinfo);
		pi.msg.header.frame_id = frame_id;
		pi.msg.width           = width;
		pi.msg.height          = height;
		pi.msg.is_dense        = is_dense;
		pi.msg.fields.clear();
		pi.msg.fields.resize(fieldinfo.size());
		for (unsigned int i = 0; i < fieldinfo.size(); ++i) {
			pi.msg.fields[i].name     = fieldinfo[i].name;
			pi.msg.fields[i].offset   = fieldinfo[i].offset;
			pi.msg.fields[i].datatype = fieldinfo[i].datatype;
			pi.msg.fields[i].count    = fieldinfo[i].count;
		}

		fawkes_pubs_[*p] = pi;
	}
}

void
ROS2PointCloudThread::fawkes_pointcloud_publish_to_ros()
{
	std::map<std::string, PublisherInfo>::iterator p;
	for (p = fawkes_pubs_.begin(); p != fawkes_pubs_.end(); ++p) {
		PublisherInfo &pi = p->second;
		if (pi.pub.getNumSubscribers() > 0 && pcl_manager->exists_pointcloud(p->first.c_str())) {
			unsigned int width, height;
			void *       point_data;
			size_t       point_size, num_points;
			fawkes::Time time;
			fawkes::Time now(time);
			std::string  frame_id;
			adapter_->get_data(
			  p->first, frame_id, width, height, time, &point_data, point_size, num_points);

			if (pi.last_sent != time) {
				pi.last_sent = time;

				size_t data_size = point_size * num_points;
				pi.msg.data.resize(data_size);
				memcpy(&pi.msg.data[0], point_data, data_size);

				pi.msg.width             = width;
				pi.msg.height            = height;
				pi.msg.header.frame_id   = frame_id;
				pi.msg.header.stamp.sec  = time.get_sec();
				pi.msg.header.stamp.nsec = time.get_nsec();
				pi.msg.point_step        = point_size;
				pi.msg.row_step          = point_size * pi.msg.width;

				pi.pub->publish(pi.msg);
				//} else {
				// logger->log_debug(name(), "No update for %s, not sending", p->first.c_str());
			}
		} else {
			if (pcl_manager->exists_pointcloud(p->first.c_str())) {
				adapter_->close(p->first);
			}
		}
	}
}

void
ROS2PointCloudThread::ros_pointcloud_on_data_msg(const sensor_msgs::msg::PointCloud2ConstPtr &msg,
                                                const std::string &                     topic_name)
{
	// if this is the first time, I need the meta infos, what point-type is send
	if (!pcl_manager->exists_pointcloud(topic_name.c_str())) {
		bool r = false, i = false;
		for (const sensor_msgs::PointField &field : msg->fields) {
			//      logger->log_info(name(), "%s: %s", topic_name.c_str(), field.name.c_str());
			if (0 == field.name.compare("r")) {
				r = true;
			}
			if (0 == field.name.compare("i")) {
				i = true;
			}
		}
		if (!r && !i) {
			logger->log_info(name(), "Adding %s with type XYZ ROS -> FAWKES", topic_name.c_str());
			add_pointcloud<pcl::PointXYZ>(msg, topic_name);
		} else if (r && !i) {
			logger->log_info(name(), "Adding %s with type XYZRGB ROS -> FAWKES", topic_name.c_str());
			add_pointcloud<pcl::PointXYZRGB>(msg, topic_name);
		} else if (!r && i) {
			logger->log_info(name(), "Adding %s with type XYRI ROS -> FAWKES", topic_name.c_str());
			add_pointcloud<pcl::PointXYZI>(msg, topic_name);
		} else {
			logger->log_error(name(), "%s: can't detect point type, using XYZ", topic_name.c_str());
			add_pointcloud<pcl::PointXYZ>(msg, topic_name);
		}
	}

	// copy data
	const pcl_utils::StorageAdapter *sa = pcl_manager->get_storage_adapter(topic_name.c_str());
	if (sa->is_pointtype<pcl::PointXYZ>()) {
		update_pointcloud<pcl::PointXYZ>(msg, topic_name);
	} else if (sa->is_pointtype<pcl::PointXYZRGB>()) {
		update_pointcloud<pcl::PointXYZRGB>(msg, topic_name);
	} else if (sa->is_pointtype<pcl::PointXYZI>()) {
		update_pointcloud<pcl::PointXYZI>(msg, topic_name);
	} else {
		logger->log_error(name(), "Can't detect cloud type");
	}

	ros_pointcloud_check_for_listener_in_fawkes();
}
