
/***************************************************************************
 *  pcl_db_retrieve_thread.h - Restore and retrieve point clouds from MongoDB
 *
 *  Created: Thu Aug 22 12:04:15 2013
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

#ifndef __PLUGINS_PERCEPTION_PCL_DB_RETRIEVE_PCL_DB_RETRIEVE_THREAD_H_
#define __PLUGINS_PERCEPTION_PCL_DB_RETRIEVE_PCL_DB_RETRIEVE_THREAD_H_

#include "pcl_db_retrieve_pipeline.h"

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/pointcloud.h>
#include <aspect/tf.h>
#include <plugins/mongodb/aspect/mongodb.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace fawkes {
  class PclDatabaseRetrieveInterface;
  class BlackBoardOnMessageWaker;
}


class PointCloudDBRetrieveThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::MongoDBAspect,
  public fawkes::TransformAspect,
  public fawkes::PointCloudAspect
{
 public:
  PointCloudDBRetrieveThread();
  virtual ~PointCloudDBRetrieveThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // members
  fawkes::PclDatabaseRetrieveInterface *retrieve_if_;
  fawkes::BlackBoardOnMessageWaker     *msg_waker_;

  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZRGB> > foutput_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_;

  fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZRGB> > foriginal_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_;

  std::string cfg_database_;
  std::string cfg_output_id_;
  std::string cfg_original_id_;

  PointCloudDBRetrievePipeline<pcl::PointXYZ>     *pl_xyz_;
  PointCloudDBRetrievePipeline<pcl::PointXYZRGB>  *pl_xyzrgb_;
};

#endif
