
/***************************************************************************
 *  pcl_db_store_thread.cpp - Store point clouds to MongoDB
 *
 *  Created: Mon May 05 14:26:22 2014
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

#include "pcl_db_store_thread.h"
#include <interfaces/PclDatabaseStoreInterface.h>

#include <blackboard/utils/on_message_waker.h>
#include <pcl_utils/pcl_adapter.h>

// from MongoDB
#include <mongo/client/dbclient.h>
#include <mongo/client/gridfs.h>

#define CFG_PREFIX "/perception/pcl-db/"

using namespace fawkes;
using namespace mongo;

/** @class PointCloudDBStoreThread "pcl_db_store_thread.h"
 * Thread to store point clouds from database on request.
 * @author Tim Niemueller
 */


/** Constructor. */
PointCloudDBStoreThread::PointCloudDBStoreThread()
  : Thread("PointCloudDBStoreThread", Thread::OPMODE_WAITFORWAKEUP),
    MongoDBAspect("default")
{
}


/** Destructor. */
PointCloudDBStoreThread::~PointCloudDBStoreThread()
{
}


void
PointCloudDBStoreThread::init()
{
  store_if_  = NULL;
  adapter_   = NULL;
  msg_waker_ = NULL;

  cfg_database_    = config->get_string(CFG_PREFIX"database-name");

  adapter_ = new PointCloudAdapter(pcl_manager, logger);

  try {
    store_if_ =
      blackboard->open_for_writing<PclDatabaseStoreInterface>("PCL Database Store");

    msg_waker_ = new BlackBoardOnMessageWaker(blackboard, store_if_, this);
  } catch (Exception &e) {
    finalize();
    throw;
  }
}

void
PointCloudDBStoreThread::finalize()
{
  delete msg_waker_;
  blackboard->close(store_if_);
  delete adapter_;
}


void
PointCloudDBStoreThread::loop()
{
  if (store_if_->msgq_empty()) return;

  if (PclDatabaseStoreInterface::StoreMessage *msg =
        store_if_->msgq_first_safe(msg))
  {
    store_if_->set_final(false);
    store_if_->set_msgid(msg->id());
    store_if_->set_error("");
    store_if_->write();

    std::string msg_database   = msg->database();
    std::string msg_collection = msg->collection();

    bool store = true;
    std::string errmsg;
    std::string database = (msg_database != "") ? msg_database : cfg_database_;
    std::string collection = database + ".";
    if (msg_collection == "") {
      collection += "pcls";
    } else if (msg_collection.find("fs.") == 0) {
      errmsg = "Passed in collection uses GridFS namespace";
      store  = false;
    } else {
      collection += msg->collection();
    }

    if (store)  store_pointcloud(msg->pcl_id(), database, collection, errmsg);

    store_if_->set_error(errmsg.c_str());
    store_if_->set_final(true);
    store_if_->write();
  } else {
    logger->log_warn(name(), "Unhandled message received");
  }
  store_if_->msgq_pop();
}


bool
PointCloudDBStoreThread::store_pointcloud(std::string pcl_id, std::string database,
					  std::string collection, std::string &errmsg)
{
  if ( ! pcl_manager->exists_pointcloud(pcl_id.c_str())) {
    errmsg = "PointCloud does not exist";
    return false;
  }

  std::string frame_id;
  unsigned int width, height;
  bool is_dense;
  void *point_data;
  size_t point_size, num_points;
  fawkes::Time time;
  PointCloudAdapter::V_PointFieldInfo fieldinfo;

  adapter_->get_data_and_info(pcl_id, frame_id, is_dense, width, height, time,
			      fieldinfo, &point_data, point_size, num_points);

  size_t data_size = point_size * num_points;

  BSONObjBuilder document;
  document.append("timestamp", (long long) time.in_msec());
  BSONObjBuilder subb(document.subobjStart("pointcloud"));
  subb.append("frame_id", frame_id);
  subb.append("is_dense", is_dense);
  subb.append("width", width);
  subb.append("height", height);
  subb.append("point_size", (unsigned int)point_size);
  subb.append("num_points", (unsigned int)num_points);

  GridFS gridfs(*mongodb_client, database);

  std::stringstream name;
  name << "pcl_" << time.in_msec();
  subb.append("data", gridfs.storeFile((char*) point_data, data_size, name.str()));

  BSONArrayBuilder subb2(subb.subarrayStart("field_info"));
  for (unsigned int i = 0; i < fieldinfo.size(); i++) {
    BSONObjBuilder fi(subb2.subobjStart());
    fi.append("name", fieldinfo[i].name);
    fi.append("offset", fieldinfo[i].offset);
    fi.append("datatype", fieldinfo[i].datatype);
    fi.append("count", fieldinfo[i].count);
    fi.doneFast();
  }
  subb2.doneFast();
  subb.doneFast();
  try {
    mongodb_client->insert(collection, document.obj());
  } catch (mongo::DBException &e) {
    logger->log_warn(this->name(), "Failed to insert into %s: %s",
		     collection.c_str(), e.what());
    errmsg = e.what();
    return false;
  }

  return true;
}
