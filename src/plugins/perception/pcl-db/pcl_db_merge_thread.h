
/***************************************************************************
 *  pcl_db_merge_thread.h - Restore and merge point clouds from MongoDB
 *
 *  Created: Wed Nov 28 10:53:14 2012 (Freiburg)
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

#ifndef _PLUGINS_PERCEPTION_PCL_DB_MERGE_PCL_DB_MERGE_THREAD_H_
#define _PLUGINS_PERCEPTION_PCL_DB_MERGE_PCL_DB_MERGE_THREAD_H_

#include "pcl_db_merge_pipeline.h"

#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/pointcloud.h>
#include <aspect/tf.h>
#include <core/threading/thread.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plugins/mongodb/aspect/mongodb.h>

namespace fawkes {
class PclDatabaseMergeInterface;
class BlackBoardOnMessageWaker;
#ifdef USE_TIMETRACKER
class TimeTracker;
#endif
} // namespace fawkes

class PointCloudDBMergeThread : public fawkes::Thread,
                                public fawkes::ClockAspect,
                                public fawkes::LoggingAspect,
                                public fawkes::ConfigurableAspect,
                                public fawkes::BlackBoardAspect,
                                public fawkes::MongoDBAspect,
                                public fawkes::TransformAspect,
                                public fawkes::PointCloudAspect
{
public:
	PointCloudDBMergeThread();
	virtual ~PointCloudDBMergeThread();

	virtual void init();
	virtual void loop();
	virtual void finalize();

	/** Stub to see name in backtrace for easier debugging. @see Thread::run() */
protected:
	virtual void
	run()
	{
		Thread::run();
	}

private: // members
	fawkes::PclDatabaseMergeInterface *merge_if_;
	fawkes::BlackBoardOnMessageWaker  *msg_waker_;

	fawkes::RefPtr<pcl::PointCloud<pcl::PointXYZRGB>> foutput_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr            output_;

	std::string cfg_database_;
	std::string cfg_output_id_;

	PointCloudDBMergePipeline<pcl::PointXYZ>    *pl_xyz_;
	PointCloudDBMergePipeline<pcl::PointXYZRGB> *pl_xyzrgb_;
};

#endif
