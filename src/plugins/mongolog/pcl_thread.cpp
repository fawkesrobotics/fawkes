
/***************************************************************************
 *  pcl_thread.cpp - Thread to log point clouds to MongoDB
 *
 *  adapted from ros/pcl_thread.cpp
 *
 *  Created: Mon Nov 07 02:58:40 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
 *  Modified: Thu Jul 12 09:51:00 2012 by Bastian Klingen
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

// from MongoDB
#include <mongo/client/dbclient.h>

using namespace fawkes;
using namespace mongo;

/** @class MongoLogPointCloudThread "pcl_thread.h"
 * Thread to store point clouds to MongoDB.
 * @author Tim Niemueller
 * @author Bastian Klingen
 */

/** Constructor. */
MongoLogPointCloudThread::MongoLogPointCloudThread()
  : Thread("MongoLogPointCloudThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

/** Destructor. */
MongoLogPointCloudThread::~MongoLogPointCloudThread()
{
}



void
MongoLogPointCloudThread::init()
{
  __database = "fflog";
  try {
    __database = config->get_string("/plugins/mongolog/database");
  } catch (Exception &e) {
    logger->log_info(name(), "No database configured, writing to %s",
		     __database.c_str());
  }
  __collection = __database + ".pointclouds";
  __mongodb    = mongodb_client;

  __adapter = new MongoLogPointCloudAdapter(pcl_manager, logger);

  std::vector<std::string> pcls = pcl_manager->get_pointcloud_list();

  std::vector<std::string>::iterator p;
  for (p = pcls.begin(); p != pcls.end(); ++p) {
    PointCloudInfo pi;

    logger->log_info(name(), "MongoLog of point cloud %s", p->c_str());

    std::string frame_id;
    unsigned int width, height;
    bool is_dense;
    MongoLogPointCloudAdapter::V_PointFieldInfo fieldinfo;
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

    __pcls[*p] = pi;
  }
}


void
MongoLogPointCloudThread::finalize()
{
  delete __adapter;
}


void
MongoLogPointCloudThread::loop()
{
  std::map<std::string, PointCloudInfo>::iterator p;
  for (p = __pcls.begin(); p != __pcls.end(); ++p) {
    PointCloudInfo &pi = p->second;
      unsigned int width, height;
      void *point_data;
      size_t point_size, num_points;
      fawkes::Time time;
      __adapter->get_data(p->first, width, height, time,
                          &point_data, point_size, num_points);
      size_t data_size = point_size * num_points;
      pi.msg.data.resize(data_size);

      if (pi.last_sent != time) {
        pi.last_sent = time;

        BSONObjBuilder document;
        document.appendTimestamp("timestamp", time.in_msec());
        BSONArrayBuilder subb(document.subobjStart("pointcloud"));
        document.append("frame_id", pi.msg.header.frame_id);
        document.append("is_dense", pi.msg.is_dense);
        document.append("width", width);
        document.append("height", height);
        document.append("point_size", (unsigned int)point_size);
        document.append("num_points", (unsigned int)num_points);
        document.appendBinData("point_data", (int) data_size, BinDataGeneral, point_data);
        BSONArrayBuilder subb2(document.subarrayStart("field_info"));
        for (unsigned int i = 0; i < pi.msg.fields.size(); i++) {
          BSONObjBuilder fi(document.subobjStart("field_info_"+i));
          document.append("name", pi.msg.fields[i].name);
          document.append("offset", pi.msg.fields[i].offset);
          document.append("datatype", pi.msg.fields[i].datatype);
          document.append("count", pi.msg.fields[i].count);
          fi.doneFast();
        }
        subb2.doneFast();
        subb.doneFast();
         __mongodb->insert(__collection, document.obj());

    } else {
      __adapter->close(p->first);
    }
  }
}
