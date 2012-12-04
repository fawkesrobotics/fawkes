
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


#include <unistd.h>

using namespace std;
using namespace fawkes;

#define CFG_PREFIX "/perception/pcl-db-merge/"

/** @class PointCloudDBMergeThread "tabletop_objects_thread.h"
 * Main thread of tabletop objects plugin.
 * @author Tim Niemueller
 */


/** Constructor. */
PointCloudDBMergeThread::PointCloudDBMergeThread()
  : Thread("PointCloudDBMergeThread", Thread::OPMODE_CONTINUOUS)
{
}


/** Destructor. */
PointCloudDBMergeThread::~PointCloudDBMergeThread()
{
}


void
PointCloudDBMergeThread::init()
{
  foutput_ = new pcl::PointCloud<pcl::PointXYZRGB>();
  //foutput_->header.frame_id = finput_->header.frame_id;
  foutput_->is_dense = false;
  pcl_manager->add_pointcloud<pcl::PointXYZRGB>("db-merged", foutput_);
  output_ = pcl_utils::cloudptr_from_refptr(foutput_);

  cfg_database_name_ = "fflog";
  cfg_global_frame_  = "/map";

  
  pl_xyz_ =
    new PointCloudDBMergePipeline<pcl::PointXYZ>(mongodb_client,
						 cfg_database_name_,
						 cfg_global_frame_,
						 output_, logger);

  pl_xyzrgb_ =
    new PointCloudDBMergePipeline<pcl::PointXYZRGB>(mongodb_client,
						    cfg_database_name_,
						    cfg_global_frame_,
						    output_, logger);
}


void
PointCloudDBMergeThread::once()
{
  //loop();
}

void
PointCloudDBMergeThread::finalize()
{
  delete pl_xyz_;
  delete pl_xyzrgb_;

  output_.reset();
  pcl_manager->remove_pointcloud("db-merged");
  foutput_.reset();
}


void
PointCloudDBMergeThread::loop()
{
  std::vector<long long> times;
  times.push_back(1354200347715);
  times.push_back(1354200406578);
  times.push_back(1354200473345);

  std::string collection = "PointClouds.openni_pointcloud_xyz";

  if (pl_xyz_->applicable(times, collection)) {
    pl_xyz_->merge(times, collection);
  } else if (pl_xyzrgb_->applicable(times, collection)) {
    pl_xyzrgb_->merge(times, collection);
  } else {
    logger->log_warn(name(), "No applicable merging pipeline known");
  }

  foutput_->header.frame_id = cfg_global_frame_;
  Time now(clock);
  pcl_utils::set_time(foutput_, now);

  usleep(1000000);
}

