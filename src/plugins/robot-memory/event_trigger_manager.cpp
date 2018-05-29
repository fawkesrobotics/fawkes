/***************************************************************************
 *  event_trigger_manager.cpp - Manager to realize triggers on events in the robot memory
 *    
 *
 *  Created: 3:53:46 PM 2016
 *  Copyright  2016  Frederik Zwilling
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


#include "event_trigger_manager.h"
#include <boost/bind.hpp>

using namespace fawkes;
using namespace mongo;


/** @class EventTriggerManager  event_trigger_manager.h
 * Manager to realize triggers on events in the robot memory
 * @author Frederik Zwilling
 */

/**
 * Constructor for class managing EventTriggers
 * @param logger Logger
 * @param config Configuration
 * @param mongo_connection_manager MongoDBConnCreator
 */
EventTriggerManager::EventTriggerManager(Logger* logger, Configuration* config,
                                         MongoDBConnCreator* mongo_connection_manager)
  : cfg_debug_(false)
{
  logger_ = logger;
  config_ = config;
  mongo_connection_manager_ = mongo_connection_manager;

  con_local_ = mongo_connection_manager_->create_client("robot-memory-local");
  if (config_->exists("/plugins/mongodb/clients/robot-memory-distributed/enabled") &&
      config_->get_bool("/plugins/mongodb/clients/robot-memory-distributed/enabled"))
  {
    con_replica_ = mongo_connection_manager_->create_client("robot-memory-distributed");
  }

  // create connections to running mongod instances because only there
  std::string local_db = config_->get_string("/plugins/robot-memory/database");
  dbnames_local_.push_back(local_db);
  dbnames_distributed_ = std::move(config_->get_strings("/plugins/robot-memory/distributed-db-names"));

  mutex_ = new Mutex();

  try {
    cfg_debug_ = config->get_bool("/plugins/robot-memory/more-debug-output");
  } catch (...) {}
}

EventTriggerManager::~EventTriggerManager()
{
	for(EventTrigger *trigger : triggers) {
		delete trigger;
	}
  mongo_connection_manager_->delete_client(con_local_);
  mongo_connection_manager_->delete_client(con_replica_);
}

void EventTriggerManager::check_events()
{
  //lock to be thread safe (e.g. registration during checking)
  MutexLocker lock(mutex_);

  for(EventTrigger *trigger : triggers)
  {
    while(trigger->oplog_cursor->more())
    {
      BSONObj change = trigger->oplog_cursor->next();
      //logger_->log_info(name.c_str(), "Triggering: %s", change.toString().c_str());
      //actually call the callback function
      trigger->callback(change);
    }
    if(trigger->oplog_cursor->isDead())
    {
      if (cfg_debug_)
        logger_->log_debug(name.c_str(), "Tailable Cursor is dead, requerying");
      //check if collection is local or replicated
      mongo::DBClientBase* con;
      if (std::find(dbnames_distributed_.begin(), dbnames_distributed_.end(),
                    get_db_name(trigger->ns_db)) != dbnames_distributed_.end())
      {
        con = con_replica_;
      } else {
        con = con_local_;
      }

      trigger->oplog_cursor = create_oplog_cursor(con, "local.oplog.rs", trigger->oplog_query);
    }
  }
}

/**
 * Remove a previously registered trigger
 * @param trigger Pointer to the trigger to remove
 */
void EventTriggerManager::remove_trigger(EventTrigger* trigger)
{
  triggers.remove(trigger);
  delete trigger;
}

QResCursor EventTriggerManager::create_oplog_cursor(mongo::DBClientBase* con, std::string oplog, mongo::Query query)
{
  QResCursor res = con->query(oplog , query, 0, 0, 0, QueryOption_CursorTailable);
  //Go to end of Oplog to get new updates from then on
  while(res->more())
  {
    res->next();
  }
  return res;
}

/** Split database name from namespace.
 * @param ns namespace, format db.collection
 * @return db part of @p ns
 */
std::string
EventTriggerManager::get_db_name(const std::string& ns)
{
	std::string::size_type dot_pos = ns.find(".");
	if (dot_pos == std::string::npos) {
		return "";
	} else {
		return ns.substr(0, dot_pos);
	}
}
