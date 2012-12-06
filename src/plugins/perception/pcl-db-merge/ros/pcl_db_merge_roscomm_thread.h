
/***************************************************************************
 *  pcl_db_merge_roscomm_thread.h - ROS communication for pcl-db-merge
 *
 *  Created: Thu Dec 06 13:52:27 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PERCEPTION_PCL_DB_MERGE_PCL_DB_MERGE_ROSCOMM_THREAD_H_
#define __PLUGINS_PERCEPTION_PCL_DB_MERGE_PCL_DB_MERGE_ROSCOMM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <plugins/ros/aspect/ros.h>

#include <hybris_c1_msgs/MergePointClouds.h>
#include <hybris_c1_msgs/RecordData.h>

namespace fawkes {
  class PclDatabaseMergeInterface;
  class BlackBoardOnUpdateWaker;
  class WaitCondition;
}
namespace ros {
  class ServiceServer;
}


class PointCloudDBMergeROSCommThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::ROSAspect
{
 public:
  PointCloudDBMergeROSCommThread();
  virtual ~PointCloudDBMergeROSCommThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  bool merge_cb(hybris_c1_msgs::MergePointClouds::Request  &req,
		hybris_c1_msgs::MergePointClouds::Response &resp);
  bool record_cb(hybris_c1_msgs::RecordData::Request  &req,
		 hybris_c1_msgs::RecordData::Response &resp);


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // members
  fawkes::PclDatabaseMergeInterface *merge_if_;

  fawkes::BlackBoardOnUpdateWaker *update_waker_;
  fawkes::WaitCondition *merge_waitcond_;

  ros::ServiceServer *srv_merge_;
  ros::ServiceServer *srv_record_;

  unsigned int msg_id_;
};

#endif
