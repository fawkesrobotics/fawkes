
/***************************************************************************
 *  pcl_db_merge_thread.cpp - Restore and merge point clouds from MongoDB
 *
 *  Created: Wed Nov 28 10:56:10 2012 (Freiburg)
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

#include "pcl_db_merge_thread.h"
#include <interfaces/PclDatabaseMergeInterface.h>

#include <blackboard/utils/on_message_waker.h>

using namespace fawkes;

/** @class PointCloudDBMergeThread "pcl_db_merge_thread.h"
 * Thread to merge point clouds from database on request.
 * @author Tim Niemueller
 */


/** Constructor. */
PointCloudDBMergeThread::PointCloudDBMergeThread()
  : Thread("PointCloudDBMergeThread", Thread::OPMODE_WAITFORWAKEUP),
    MongoDBAspect("default")
{
}


/** Destructor. */
PointCloudDBMergeThread::~PointCloudDBMergeThread()
{
}


void
PointCloudDBMergeThread::init()
{
  pl_xyz_ = NULL;
  pl_xyzrgb_ = NULL;
  merge_if_ = NULL;
  msg_waker_ = NULL;

  cfg_database_    = config->get_string(CFG_PREFIX"database-name");
  cfg_output_id_   = config->get_string(CFG_PREFIX_MERGE"output-pcl-id");

  foutput_ = new pcl::PointCloud<pcl::PointXYZRGB>();
  //foutput_->header.frame_id = finput_->header.frame_id;
  foutput_->is_dense = false;
  pcl_manager->add_pointcloud<pcl::PointXYZRGB>(cfg_output_id_.c_str(), foutput_);
  output_ = pcl_utils::cloudptr_from_refptr(foutput_);

  
  pl_xyz_ =
    new PointCloudDBMergePipeline<pcl::PointXYZ>(mongodb_client,
						 config, logger, tf_listener,
						 output_);

  pl_xyzrgb_ =
    new PointCloudDBMergePipeline<pcl::PointXYZRGB>(mongodb_client,
						    config, logger, tf_listener,
						    output_);

  try {
    merge_if_ =
      blackboard->open_for_writing<PclDatabaseMergeInterface>("PCL Database Merge");

    msg_waker_ = new BlackBoardOnMessageWaker(blackboard, merge_if_, this);
  } catch (Exception &e) {
    finalize();
    throw;
  }
}

void
PointCloudDBMergeThread::finalize()
{
  delete msg_waker_;
  blackboard->close(merge_if_);

  delete pl_xyz_;
  delete pl_xyzrgb_;

  output_.reset();
  pcl_manager->remove_pointcloud(cfg_output_id_.c_str());
  foutput_.reset();
}


void
PointCloudDBMergeThread::loop()
{
  std::vector<long long> times;
  std::string database;
  std::string collection;
  //= "PointClouds.openni_pointcloud_xyz";

  if (merge_if_->msgq_empty()) return;

  if (PclDatabaseMergeInterface::MergeMessage *msg =
        merge_if_->msgq_first_safe(msg))
  {
    merge_if_->set_final(false);
    merge_if_->set_msgid(msg->id());
    merge_if_->set_error("");
    merge_if_->write();

    int64_t *timestamps = msg->timestamps();
    for (size_t i = 0; i < msg->maxlenof_timestamps(); ++i) {
      if (timestamps[i] > 0) {
	times.push_back(timestamps[i]);
      }
    }
    database     =
      (strcmp(msg->database(), "") != 0) ? msg->database() : cfg_database_;
    collection   = msg->collection();
  }

  merge_if_->msgq_pop();

  /* For testing
  collection = "PointClouds.openni_pointcloud_xyz";
  times.clear();
  times.push_back(1354200347715);
  times.push_back(1354200406578);
  times.push_back(1354200473345);
  */

  if (times.empty()) {
    logger->log_warn(name(), "Called for merge from %s, but no times given",
                     collection.c_str());
    merge_if_->set_final(true);
    merge_if_->set_error("Called for merge, but no non-zero times given");
    merge_if_->write();
    return;
  }

  logger->log_info(name(), "Restoring from '%s' for the following times",
		   collection.c_str());
  for (size_t i = 0; i < times.size(); ++i) {
    logger->log_info(name(), "  %lli", times[i]);
  }

  ApplicabilityStatus st_xyz, st_xyzrgb;

  pl_xyz_->applicable(times, database, collection);
  if ((st_xyz = pl_xyz_->applicable(times, database, collection)) == APPLICABLE) {
    pl_xyz_->merge(times, database, collection);
    Time now(clock);
    pcl_utils::set_time(foutput_, now);
  } else if ((st_xyzrgb = pl_xyzrgb_->applicable(times, database, collection)) == APPLICABLE) {
    pl_xyzrgb_->merge(times, database, collection);
    Time now(clock);
    pcl_utils::set_time(foutput_, now);
  } else {
    logger->log_warn(name(), "No applicable merging pipeline known:");
    logger->log_warn(name(), "  XYZ:     %s", to_string(st_xyz));
    logger->log_warn(name(), "  XYZ/RGB: %s", to_string(st_xyzrgb));
    merge_if_->set_error("Merge failed, see pcl-db-merge log");
  }

  merge_if_->set_final(true);
  merge_if_->write();
}

