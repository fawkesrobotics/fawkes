
/***************************************************************************
 *  mongodb_log_pcl_thread.cpp - Thread to log point clouds to MongoDB
 *
 *  Created: Mon Nov 07 02:58:40 2011
 *  Copyright  2011-2017  Tim Niemueller [www.niemueller.de]
 *             2012       Bastian Klingen
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

#include "mongodb_log_pcl_thread.h"

// Fawkes
#include <core/threading/mutex_locker.h>
#include <utils/time/wait.h>

// from MongoDB
#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>

#include <fnmatch.h>
#include <unistd.h>

using namespace fawkes;
using namespace mongo;

/** @class MongoLogPointCloudThread "mongodb_log_pcl_thread.h"
 * Thread to store point clouds to MongoDB.
 * @author Tim Niemueller
 * @author Bastian Klingen
 */

/** Constructor. */
MongoLogPointCloudThread::MongoLogPointCloudThread()
  : Thread("MongoLogPointCloudThread", Thread::OPMODE_CONTINUOUS),
    MongoDBAspect("default")
{
  set_prepfin_conc_loop(true);
}

/** Destructor. */
MongoLogPointCloudThread::~MongoLogPointCloudThread()
{
}



void
MongoLogPointCloudThread::init()
{
  database_ = "fflog";
  try {
    database_ = config->get_string("/plugins/mongodb-log/database");
  } catch (Exception &e) {
    logger->log_info(name(), "No database configured, writing to %s",
		     database_.c_str());
  }

  cfg_storage_interval_ =
    config->get_float("/plugins/mongodb-log/pointclouds/storage-interval");

  cfg_chunk_size_ = 2097152; // 2 MB
  try {
    cfg_chunk_size_ = config->get_uint("/plugins/mongodb-log/pointclouds/chunk-size");
  } catch (Exception &e) {} // ignored, use default
  logger->log_info(name(), "Chunk size: %u", cfg_chunk_size_);

  cfg_flush_after_write_ = false;
  try {
    cfg_flush_after_write_ =
      config->get_uint("/plugins/mongodb-log/pointclouds/flush-after-write");
  } catch (Exception &e) {} // ignored, use default

  std::vector<std::string> includes;
  try {
    includes = config->get_strings("/plugins/mongodb-log/pointclouds/includes");
  } catch (Exception &e) {} // ignored, no include rules
  std::vector<std::string> excludes;

  try {
    excludes = config->get_strings("/plugins/mongodb-log/pointclouds/excludes");
  } catch (Exception &e) {} // ignored, no include rules

  mongodb_    = mongodb_client;
  gridfs_  = new GridFS(*mongodb_, database_);
  //gridfs_->setChunkSize(cfg_chunk_size_);

  adapter_ = new PointCloudAdapter(pcl_manager, logger);

  std::vector<std::string> pcls = pcl_manager->get_pointcloud_list();

  std::vector<std::string>::iterator p;
  std::vector<std::string>::iterator f;
  for (p = pcls.begin(); p != pcls.end(); ++p) {
    bool include = includes.empty();
    if (! include) {
      for (f = includes.begin(); f != includes.end(); ++f) {
	if (fnmatch(f->c_str(), p->c_str(), 0) != FNM_NOMATCH) {
	  logger->log_debug(name(), "Include match %s to %s", f->c_str(), p->c_str());
	  include = true;
	  break;
	}
      }
    }
    if (include) {
      for (f = excludes.begin(); f != excludes.end(); ++f) {
	if (fnmatch(f->c_str(), p->c_str(), 0) != FNM_NOMATCH) {
	  logger->log_debug(name(), "Exclude match %s to %s", f->c_str(), p->c_str());
	  include = false;
	  break;
	}
      }
    }
    if (! include) {
      logger->log_info(name(), "Excluding point cloud %s", p->c_str());
      continue;
    }

    PointCloudInfo pi;

    std::string topic_name = std::string("PointClouds.") + *p;
    size_t pos = 0;
    while ((pos = topic_name.find_first_of(" -", pos)) != std::string::npos) {
      topic_name.replace(pos, 1, "_");
      pos = pos + 1;
    }

    pi.topic_name = topic_name;

    logger->log_info(name(), "MongoLog of point cloud %s", p->c_str());

    std::string frame_id;
    unsigned int width, height;
    bool is_dense;
    PointCloudAdapter::V_PointFieldInfo fieldinfo;
    adapter_->get_info(*p, width, height, frame_id, is_dense, fieldinfo);
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

    pcls_[*p] = pi;
  }

  wait_  = new TimeWait(clock, cfg_storage_interval_ * 1000000.);
  mutex_ = new Mutex();
}


bool
MongoLogPointCloudThread::prepare_finalize_user()
{
  mutex_->lock();
  return true;
}


void
MongoLogPointCloudThread::finalize()
{
  delete adapter_;
  delete gridfs_;
  delete wait_;
  delete mutex_;
}


void
MongoLogPointCloudThread::loop()
{
  MutexLocker lock(mutex_);
  fawkes::Time loop_start(clock);
  wait_->mark_start();
  std::map<std::string, PointCloudInfo>::iterator p;
  unsigned int num_stored = 0;
  for (p = pcls_.begin(); p != pcls_.end(); ++p) {
    PointCloudInfo &pi = p->second;
    std::string frame_id;
    unsigned int width, height;
    void *point_data;
    size_t point_size, num_points;
    fawkes::Time time;
    adapter_->get_data(p->first, frame_id, width, height, time,
		       &point_data, point_size, num_points);
    size_t data_size = point_size * num_points;

    if (pi.last_sent != time) {
      pi.last_sent = time;

      fawkes::Time start(clock);

      BSONObjBuilder document;
      document.append("timestamp", (long long) time.in_msec());
      BSONObjBuilder subb(document.subobjStart("pointcloud"));
      subb.append("frame_id", pi.msg.header.frame_id);
      subb.append("is_dense", pi.msg.is_dense);
      subb.append("width", width);
      subb.append("height", height);
      subb.append("point_size", (unsigned int)point_size);
      subb.append("num_points", (unsigned int)num_points);

      std::stringstream name;
      name << pi.topic_name << "_" << time.in_msec();
      subb.append("data", gridfs_->storeFile((char*) point_data, data_size, name.str()));

      BSONArrayBuilder subb2(subb.subarrayStart("field_info"));
      for (unsigned int i = 0; i < pi.msg.fields.size(); i++) {
	BSONObjBuilder fi(subb2.subobjStart());
	fi.append("name", pi.msg.fields[i].name);
	fi.append("offset", pi.msg.fields[i].offset);
	fi.append("datatype", pi.msg.fields[i].datatype);
	fi.append("count", pi.msg.fields[i].count);
	fi.doneFast();
      }
      subb2.doneFast();
      subb.doneFast();
      collection_ = database_ + "." + pi.topic_name;
      try {
	mongodb_->insert(collection_, document.obj());
	++num_stored;
      } catch (mongo::DBException &e) {
	logger->log_warn(this->name(), "Failed to insert into %s: %s",
			 collection_.c_str(), e.what());
      }

      fawkes::Time end(clock);
      float diff = (end - &start) * 1000.;
      logger->log_debug(this->name(), "Stored point cloud %s (time %li) in %.1f ms",
			p->first.c_str(), time.in_msec(), diff);

    } else {
	logger->log_debug(this->name(), "Point cloud %s did not change",
			  p->first.c_str());
	//adapter_->close(p->first);
    }

  }
  mutex_->unlock();
  // -1 to subtract "NO PARENT" pseudo cache
  fawkes::Time loop_end(clock);
  logger->log_debug(name(), "Stored %u of %zu point clouds in %.1f ms",
		    num_stored, pcls_.size(), (loop_end - &loop_start) * 1000.);

  if (cfg_flush_after_write_) {
    // flush database
    BSONObjBuilder flush_cmd;
    BSONObj reply;
    flush_cmd.append("fsync", 1);
    flush_cmd.append("async", 1);
    mongodb_client->runCommand("admin", flush_cmd.obj(), reply);
    if (reply.hasField("ok")) {
      if (! reply["ok"].trueValue()) {
	logger->log_warn(name(), "fsync error: %s", reply["errmsg"].String().c_str());
      }
    } else {
      logger->log_warn(name(), "fsync reply has no ok field");
    }
  }
  wait_->wait();
}
