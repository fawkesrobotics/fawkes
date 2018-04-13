
/***************************************************************************
 *  pcl_db_retrieve_thread.cpp - Restore and retrieve point clouds from MongoDB
 *
 *  Created: Thu Aug 22 12:04:09 2013
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

#include "pcl_db_retrieve_thread.h"
#include <interfaces/PclDatabaseRetrieveInterface.h>

#include <blackboard/utils/on_message_waker.h>

using namespace fawkes;

/** @class PointCloudDBRetrieveThread "pcl_db_retrieve_thread.h"
 * Thread to retrieve point clouds from database on request.
 * @author Tim Niemueller
 */


/** Constructor. */
PointCloudDBRetrieveThread::PointCloudDBRetrieveThread()
  : Thread("PointCloudDBRetrieveThread", Thread::OPMODE_WAITFORWAKEUP),
    MongoDBAspect("default")
{
}


/** Destructor. */
PointCloudDBRetrieveThread::~PointCloudDBRetrieveThread()
{
}


void
PointCloudDBRetrieveThread::init()
{
  pl_xyz_ = NULL;
  pl_xyzrgb_ = NULL;
  retrieve_if_ = NULL;
  msg_waker_ = NULL;

  cfg_database_    = config->get_string(CFG_PREFIX"database-name");
  cfg_output_id_   = config->get_string(CFG_PREFIX_RETRV"output-pcl-id");
  cfg_original_id_ = config->get_string(CFG_PREFIX_RETRV"original-pcl-id");

  foutput_ = new pcl::PointCloud<pcl::PointXYZRGB>();
  foutput_->is_dense = false;
  pcl_manager->add_pointcloud<pcl::PointXYZRGB>(cfg_output_id_.c_str(), foutput_);
  output_ = pcl_utils::cloudptr_from_refptr(foutput_);

  foriginal_ = new pcl::PointCloud<pcl::PointXYZRGB>();
  foriginal_->is_dense = false;
  pcl_manager->add_pointcloud<pcl::PointXYZRGB>(cfg_original_id_.c_str(), foriginal_);
  original_ = pcl_utils::cloudptr_from_refptr(foriginal_);

  
  pl_xyz_ =
    new PointCloudDBRetrievePipeline<pcl::PointXYZ>(mongodb_client,
						    config, logger, tf_listener,
						    original_, output_);

  pl_xyzrgb_ =
    new PointCloudDBRetrievePipeline<pcl::PointXYZRGB>(mongodb_client,
						    config, logger, tf_listener,
						    original_, output_);

  try {
    retrieve_if_ =
      blackboard->open_for_writing<PclDatabaseRetrieveInterface>("PCL Database Retrieve");

    msg_waker_ = new BlackBoardOnMessageWaker(blackboard, retrieve_if_, this);
  } catch (Exception &e) {
    finalize();
    throw;
  }
}

void
PointCloudDBRetrieveThread::finalize()
{
  delete msg_waker_;
  blackboard->close(retrieve_if_);

  delete pl_xyz_;
  delete pl_xyzrgb_;

  output_.reset();
  pcl_manager->remove_pointcloud(cfg_output_id_.c_str());
  foutput_.reset();

  original_.reset();
  pcl_manager->remove_pointcloud(cfg_original_id_.c_str());
  foriginal_.reset();
}


void
PointCloudDBRetrieveThread::loop()
{
  long long timestamp = 0;
  std::vector<long long> times(1);
  std::string database;
  std::string collection;
  std::string target_frame;
  bool        original_timestamp;

  if (retrieve_if_->msgq_empty()) return;

  if (PclDatabaseRetrieveInterface::RetrieveMessage *msg =
        retrieve_if_->msgq_first_safe(msg))
  {
    retrieve_if_->set_final(false);
    retrieve_if_->set_msgid(msg->id());
    retrieve_if_->set_error("");
    retrieve_if_->write();

    timestamp    = msg->timestamp();
    times[0]     = timestamp;
    database     =
      (strcmp(msg->database(), "") != 0) ? msg->database() : cfg_database_;
    collection   = msg->collection();
    target_frame = msg->target_frame();
    original_timestamp = msg->is_original_timestamp();
  } else {
    logger->log_warn(name(), "Unhandled message received");
    retrieve_if_->msgq_pop();
    return;
  }
  retrieve_if_->msgq_pop();

  logger->log_info(name(), "Restoring from '%s' for the time %lli",
		   collection.c_str(), timestamp);

  ApplicabilityStatus st_xyz, st_xyzrgb;
  long long actual_time = 0;

  pl_xyz_->applicable(times, database, collection);
  if ((st_xyz = pl_xyz_->applicable(times, database, collection)) == APPLICABLE) {
    logger->log_info(name(), "Restoring XYZ");
    pl_xyz_->retrieve(timestamp, database, collection, target_frame, actual_time);
  } else if ((st_xyzrgb = pl_xyzrgb_->applicable(times, database, collection)) == APPLICABLE) {
    logger->log_info(name(), "Restoring XYZRGB");
    pl_xyzrgb_->retrieve(timestamp, database, collection, target_frame, actual_time);
    if (! original_timestamp) {
      Time now(clock);
      pcl_utils::set_time(foutput_, now);
    }
  } else {
    logger->log_warn(name(), "No applicable merging pipeline known:");
    logger->log_warn(name(), "  XYZ:     %s", to_string(st_xyz));
    logger->log_warn(name(), "  XYZ/RGB: %s", to_string(st_xyzrgb));
    retrieve_if_->set_error("No applicable merging pipeline known (for details see log)");
  }

  if (actual_time != 0) {
    if (original_timestamp) {
      Time now((long)actual_time);
      Time last;
      pcl_utils::get_time(foutput_, last);
      // force sending with one microsecond offset if same than last time
      if (last == now)	now += 1L;
      pcl_utils::set_time(foutput_, now);
    } else {
      Time now(clock);
      pcl_utils::set_time(foutput_, now);
    }
  }

  retrieve_if_->set_final(true);
  retrieve_if_->write();
}

