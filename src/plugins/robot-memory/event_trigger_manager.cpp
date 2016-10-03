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
#include <plugin/loader.h>

using namespace fawkes;
using namespace mongo;

EventTriggerManager::EventTriggerManager(Logger* logger, Configuration* config)
{
  logger_ = logger;
  config_ = config;

  // create connections to running mongod instances because only there
  con_local_ = new mongo::DBClientConnection();
  local_db = config_->get_string("plugins/robot-memory/database");
  std::string errmsg;
  if(!con_local_->connect("localhost:" + std::to_string(config_->get_uint("plugins/robot-memory/setup/local/port")), errmsg))
  {
    std::string err_msg = "Could not connect to mongod process: "+ errmsg;
    throw PluginLoadException("robot-memory", err_msg.c_str());
  }
  repl_set = config_->get_string("plugins/robot-memory/setup/replicated/replica-set-name");
  con_replica_ = new mongo::DBClientConnection();
  //TODO: connect to repl set instead of instance
  if(!con_replica_->connect("localhost:" + std::to_string(config_->get_uint("plugins/robot-memory/setup/replicated/port")), errmsg))
  {
    std::string err_msg = "Could not connect to replica set: "+ errmsg;
    throw PluginLoadException("robot-memory", err_msg.c_str());
  }

  //test setup
  register_trigger(mongo::fromjson("{}"), "syncedrobmem.test");

  logger_->log_info(name.c_str(), "Initialized");
}

EventTriggerManager::~EventTriggerManager()
{
}

void EventTriggerManager::check_events()
{
  while(oplog_cursor->more())
  {
    BSONObj change = oplog_cursor->next();
    logger_->log_info(name.c_str(), "Oplog has more: %s", change.toString().c_str());
  }
  if(oplog_cursor->isDead())
  {
    logger_->log_info(name.c_str(), "Tailable Cursor is dead, requerying");
    oplog_cursor = create_oplog_cursor(con_replica_, "local.oplog.rs", oplog_query);
  }
}

void EventTriggerManager::register_trigger(mongo::Query query, std::string collection)
{
  logger_->log_info(name.c_str(), "Registering Trigger");

  oplog_query = query;
  oplog_query.readPref(ReadPreference_Nearest, BSONArray());
  oplog_collection = collection;

  //TODO: check if collection is local or replicated
  oplog_cursor = create_oplog_cursor(con_replica_, "local.oplog.rs", oplog_query);
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
