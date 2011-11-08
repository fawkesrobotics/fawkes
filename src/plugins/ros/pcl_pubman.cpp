
/***************************************************************************
 *  pcl_pubman.cpp - PCL exchange publisher manager
 *
 *  Created: Mon Nov 07 22:55:26 2011
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

#include "pcl_pubman.h"
#include <core/exception.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/node_handle.h>

using namespace fawkes;

struct RosPointCloudPublisherManager::PublisherInfo {
  ros::Publisher           pub;
  sensor_msgs::PointCloud2 msg;
};

/** Constructor. */
RosPointCloudPublisherManager::RosPointCloudPublisherManager(fawkes::LockPtr<ros::NodeHandle> rosnode)
{
  __rosnode = rosnode;
}


void
RosPointCloudPublisherManager::add_publisher(std::string &id,
                                             unsigned int width,
                                             unsigned int height,
                                             std::string frame_id, bool is_dense,
                                             const std::vector<PointFieldInfo> &pfi)
{
  if (__pubs.find(id) != __pubs.end()) {
    throw Exception("Publisher %s already registered", id.c_str());
  }

  std::string topic_name = std::string("fawkes_pcls/") + id;
  std::string::size_type pos = 0;
  while ((pos = topic_name.find("-", pos)) != std::string::npos) {
    topic_name.replace(pos, 1, "_");
  }

  PublisherInfo *p = new PublisherInfo();
  __rosnode.lock();
  p->pub = __rosnode->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
  __rosnode.unlock();

  if (width == 0 && height == 0) {
    p->msg.width  = width;
    p->msg.height = 1;
  } else {
    p->msg.height = height;
    p->msg.width  = width;
  }

  p->msg.fields.clear();
  p->msg.fields.resize(pfi.size());
  for (unsigned int i = 0; i < pfi.size(); ++i) {
    sensor_msgs::PointField f;
    p->msg.fields[i].name     = pfi[i].name;
    p->msg.fields[i].offset   = pfi[i].offset;
    p->msg.fields[i].datatype = pfi[i].datatype;
    p->msg.fields[i].count    = pfi[i].count;
  }

  p->msg.header.frame_id   = frame_id;
  p->msg.is_dense          = is_dense;

  __pubs[id] = p;
}


void
RosPointCloudPublisherManager::remove_publisher(std::string &id)
{
  if (__pubs.find(id) != __pubs.end()) {
    __pubs.erase(id);
  }
}

 
void
RosPointCloudPublisherManager::publish(std::string &id, fawkes::Time &time,
                                       void *point_data, size_t point_size,
                                       size_t num_points)
{
  if (__pubs.find(id) == __pubs.end()) {
    throw Exception("No publisher for %s has been registered", id.c_str());
  }

  PublisherInfo *p = __pubs[id];

  // Fill point cloud binary data (padding and all)
  size_t data_size = point_size * num_points;
  p->msg.data.resize(data_size);
  memcpy (&p->msg.data[0], point_data, data_size);

  p->msg.header.stamp.sec  = time.get_sec();
  p->msg.header.stamp.nsec = time.get_nsec();
  p->msg.point_step        = point_size;
  p->msg.row_step          = point_size * p->msg.width;
}
