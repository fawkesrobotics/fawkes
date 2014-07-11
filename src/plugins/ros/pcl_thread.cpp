
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
#include <sensor_msgs/PointCloud2.h>

using namespace fawkes;

/** @class RosPointCloudThread "pcl_thread.h"
 * Thread to exchange point clouds between Fawkes and ROS.
 * @author Tim Niemueller
 */

/** Constructor. */
RosPointCloudThread::RosPointCloudThread()
  : Thread("RosPointCloudThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
}

/** Destructor. */
RosPointCloudThread::~RosPointCloudThread()
{
}



void
RosPointCloudThread::init()
{
  //__pubman = new RosPointCloudPublisherManager(rosnode);
  __adapter = new PointCloudAdapter(pcl_manager, logger);

  std::vector<std::string> pcls = pcl_manager->get_pointcloud_list();

  std::vector<std::string>::iterator p;
  for (p = pcls.begin(); p != pcls.end(); ++p) {

    std::string topic_name = std::string("fawkes_pcls/") + *p;
    std::string::size_type pos = 0;
    while ((pos = topic_name.find("-", pos)) != std::string::npos) {
      topic_name.replace(pos, 1, "_");
    }

    PublisherInfo pi;
    pi.pub = rosnode->advertise<sensor_msgs::PointCloud2>(topic_name, 1);

    logger->log_info(name(), "Publishing point cloud %s at %s", p->c_str(), topic_name.c_str());

    std::string frame_id;
    unsigned int width, height;
    bool is_dense;
    PointCloudAdapter::V_PointFieldInfo fieldinfo;
    __adapter->get_info(*p, width, height, frame_id, is_dense, fieldinfo);
    pi.msg.header.frame_id = frame_id;
    pi.msg.width = width;
    pi.msg.height = height;
    pi.msg.is_dense = is_dense;
    pi.msg.fields.clear();
    pi.msg.fields.resize(fieldinfo.size());
    for (unsigned int i = 0; i < fieldinfo.size(); ++i) {
      pi.msg.fields[i].name     = fieldinfo[i].name;
      pi.msg.fields[i].offset   = fieldinfo[i].offset;
      pi.msg.fields[i].datatype = fieldinfo[i].datatype;
      pi.msg.fields[i].count    = fieldinfo[i].count;
    }

    __pubs[*p] = pi;
  }
}


void
RosPointCloudThread::finalize()
{
  delete __adapter;
}


void
RosPointCloudThread::loop()
{
  std::map<std::string, PublisherInfo>::iterator p;
  for (p = __pubs.begin(); p != __pubs.end(); ++p) {
    PublisherInfo &pi = p->second;
    if (pi.pub.getNumSubscribers() > 0) {
      unsigned int width, height;
      void *point_data;
      size_t point_size, num_points;
      fawkes::Time time;
      fawkes::Time now(time);
      std::string frame_id;
      __adapter->get_data(p->first, frame_id, width, height, time,
                          &point_data, point_size, num_points);

      if (pi.last_sent != time) {
        pi.last_sent = time;

        size_t data_size = point_size * num_points;
        pi.msg.data.resize(data_size);
        memcpy (&pi.msg.data[0], point_data, data_size);

        pi.msg.width             = width;
        pi.msg.height            = height;
	pi.msg.header.frame_id   = frame_id;
        pi.msg.header.stamp.sec  = time.get_sec();
        pi.msg.header.stamp.nsec = time.get_nsec();
        pi.msg.point_step        = point_size;
        pi.msg.row_step          = point_size * pi.msg.width;

        pi.pub.publish(pi.msg);
      //} else {
        // logger->log_debug(name(), "No update for %s, not sending", p->first.c_str());
      }
    } else {
      __adapter->close(p->first);
    }
  }
}
