
/***************************************************************************
 *  pcl_pubman.h - PCL exchange publisher manager
 *
 *  Created: Mon Nov 07 22:54:37 2011
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

#ifndef __PLUGINS_ROS_PCL_PUBMAN_H_
#define __PLUGINS_ROS_PCL_PUBMAN_H_

#include <map>
#include <string>
#include <stdint.h>
#include <utils/time/time.h>
#include <core/utils/lockptr.h>
#include <ros/publisher.h>

class RosPointCloudPublisherManager
{
 public:
  typedef struct {
    std::string name;      ///< Name of field
    uint32_t offset;    ///< Offset from start of point struct
    uint8_t  datatype;  ///< Datatype enumeration see above
    uint32_t count;     ///< How many elements in field
  } PointFieldInfo;

  RosPointCloudPublisherManager(fawkes::LockPtr<ros::NodeHandle> rosnode);

  void add_publisher(std::string &id,
                     unsigned int width, unsigned int height,
                     std::string frame_id, bool is_dense,
                     const std::vector<PointFieldInfo> &pfi);

  void remove_publisher(std::string &id);

  void publish(std::string &id,  fawkes::Time &time,
               void *point_date, size_t point_size, size_t num_points);

 private:
  fawkes::LockPtr<ros::NodeHandle> __rosnode;
  struct PublisherInfo;
  std::map<std::string, PublisherInfo *> __pubs;
};

#endif
