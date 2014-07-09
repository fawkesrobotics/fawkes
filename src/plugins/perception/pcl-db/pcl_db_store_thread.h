
/***************************************************************************
 *  pcl_db_store_thread.h - Store point clouds to MongoDB
 *
 *  Created: Mon May 05 14:26:15 2014
 *  Copyright  2012-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PERCEPTION_PCL_DB_STORE_PCL_DB_STORE_THREAD_H_
#define __PLUGINS_PERCEPTION_PCL_DB_STORE_PCL_DB_STORE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/pointcloud.h>
#include <plugins/mongodb/aspect/mongodb.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace fawkes {
  class PclDatabaseStoreInterface;
  class BlackBoardOnMessageWaker;
}
class PointCloudAdapter;

class PointCloudDBStoreThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::MongoDBAspect,
  public fawkes::PointCloudAspect
{
 public:
  PointCloudDBStoreThread();
  virtual ~PointCloudDBStoreThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  bool store_pointcloud(std::string pcl_id, std::string database,
			std::string collection, std::string &errmsg);


 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: // members
  fawkes::PclDatabaseStoreInterface *store_if_;
  fawkes::BlackBoardOnMessageWaker  *msg_waker_;
  PointCloudAdapter                 *adapter_;

  std::string cfg_input_id_;
  std::string cfg_database_;
};

#endif
