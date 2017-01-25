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

EventTriggerManager::EventTriggerManager(Logger* logger, Configuration* config)
{
  logger_ = logger;
  config_ = config;
  distributed_ = config_->get_bool("plugins/robot-memory/setup/distributed");

  // create connections to running mongod instances because only there
  con_local_ = new mongo::DBClientConnection();
  local_db = config_->get_string("plugins/robot-memory/database");
  std::string errmsg;
  if(!con_local_->connect("localhost:" + std::to_string(config_->get_uint("plugins/robot-memory/setup/local/port")), errmsg))
  {
    std::string err_msg = "Could not connect to mongod process: "+ errmsg;
    throw PluginLoadException("robot-memory", err_msg.c_str());
  }
  repl_set_dist = config_->get_string("plugins/robot-memory/setup/replicated/replica-set-name");
  repl_set_local = config_->get_string("plugins/robot-memory/setup/local/replica-set-name");
  if(distributed_)
  {
    con_replica_ = new mongo::DBClientConnection();
    if(!con_replica_->connect("localhost:" + std::to_string(config_->get_uint("plugins/robot-memory/setup/replicated/port")), errmsg))
    {
      std::string err_msg = "Could not connect to replica set: "+ errmsg;
      throw PluginLoadException("robot-memory", err_msg.c_str());
    }
  }

  mutex_ = new Mutex();

  logger_->log_debug(name.c_str(), "Initialized");
}

EventTriggerManager::~EventTriggerManager()
{
  for(EventTrigger *trigger : triggers)
    {
      delete trigger;
    }
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
      logger_->log_debug(name.c_str(), "Tailable Cursor is dead, requerying");
      //check if collection is local or replicated
      mongo::DBClientConnection* con;
      if(trigger->oplog_collection.find(repl_set_dist) == 0)
      {
        con = con_replica_;
      }
      else
      {
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

QResCursor EventTriggerManager::create_oplog_cursor(mongo::DBClientConnection* con, std::string oplog, mongo::Query query)
{
  QResCursor res = con->query(oplog , query, 0, 0, 0, QueryOption_CursorTailable);
  //Go to end of Oplog to get new updates from then on
  while(res->more())
  {
    res->next();
  }
  return res;
}

