/***************************************************************************
 *  robot_memory.cpp - Class for storing and querying information in the RobotMemory
 *    
 *
 *  Created: Aug 23, 2016 1:34:32 PM 2016
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
#include "robot_memory.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <memory>
#include <string>

// from MongoDB
#include <mongo/client/dbclient.h>

using namespace mongo;
using namespace fawkes;

RobotMemory::RobotMemory(fawkes::Configuration* config, fawkes::Logger* logger,
   fawkes::Clock* clock, mongo::DBClientBase* mongodb_client,
   fawkes::BlackBoard* blackboard)
{
  mutex_ = new Mutex();
  config_ = config;
  logger_ = logger;
  clock_ = clock;
  mongodb_client_ = mongodb_client;
  blackboard_ = blackboard;
  debug_ = false;
}

RobotMemory::~RobotMemory()
{
  delete mutex_;
  blackboard_->close(rm_if_);
}

void RobotMemory::init()
{
  //load config values
  logger_->log_info(name_, "Started RobotMemory");
  default_collection_ = "fawkes.msglog";
  try {
    default_collection_ = config_->get_string("/plugins/robot-memory/default-collection");
  } catch (Exception &e) {}
  try {
    debug_ = config_->get_bool("/plugins/robot-memory/more-debug-output");
  } catch (Exception &e) {}

  //init blackboard interface
  rm_if_ = blackboard_->open_for_writing<RobotMemoryInterface>(config_->get_string("/plugins/robot-memory/interface-name").c_str());
  rm_if_->set_error("");
  rm_if_->set_result("");
  rm_if_->write();

  if(debug_)
  {
    logger_->log_info(name_, "Initialized RobotMemory");
  }
}

QResCursor RobotMemory::query(std::string query_string, std::string collection)
{
  if(collection == "")
  {
    collection = default_collection_;
  }
  if(debug_)
  {
    logger_->log_info(name_, "Executing Query %s on collection %s", query_string.c_str(), collection);
  }

  //only one query at a time
  MutexLocker lock(mutex_);

  //get query from string
  Query query;
  try{
    query = Query(query_string);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Can't parse query_string '%s'\n Exception: %s",
          query_string.c_str(), e.toString().c_str());
    }
    return NULL;
  }

//  //introspect query
//  log(query, "executing query:");

//  //TODO: computation on demand
//  //check if virtual knowledge is queried
//  //rename field in query
//  if(query.getFilter().hasField("class")){
//    set_fields(query, std::string("{type:\"") +
//               query.getFilter()["class"].String() + "\"}");
//    remove_field(query, "class");
//  }
//
//  if(query.getFilter().hasField("bbinterface")){
//    collection = config->get_string("plugins/robot-memory/blackboard-collection");
//    gen_blackboard_data(query.getFilter()["bbinterface"].String());
//  }

  //actually execute query
  QResCursor cursor;
  try{
    cursor = mongodb_client_->query(collection, query);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Error for query %s\n Exception: %s",
          query_string.c_str(), e.toString().c_str());
    }
    return NULL;
  }
  return cursor;
}

int RobotMemory::insert(std::string insert_string, std::string collection)
{
  if(collection == "")
  {
    collection = default_collection_;
  }
  if(debug_)
  {
    logger_->log_info(name_, "Executing Query %s on collection %s", insert_string.c_str(), collection);
  }

  //only one query at a time
  MutexLocker lock(mutex_);

  //get query from string
  BSONObj obj;
  try{
    obj = fromjson(insert_string);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Can't parse insert_string '%s'\n Exception: %s",
          insert_string.c_str(), e.toString().c_str());
    }
    return 0;
  }

  set_fields(obj, "{type: \"test\"}");

  //actually execute insert
  try{
    mongodb_client_->insert(collection, obj);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Error for insert %s\n Exception: %s",
          insert_string.c_str(), e.toString().c_str());
    }
    return 0;
  }
  //return success
  return 1;
}

int RobotMemory::update(std::string query_string, std::string update_string,
                                    std::string collection)
{
  if(collection == "")
  {
    collection = default_collection_;
  }
  if(debug_)
  {
    logger_->log_info(name_, "Executing Update %s for query %s on collection %s",
        update_string.c_str(), query_string.c_str(), collection);
  }

  //only one query at a time
  MutexLocker lock(mutex_);

  //get query from string
  Query query;
  try{
    query = Query(query_string);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Can't parse query_string '%s'\n Exception: %s",
          query_string.c_str(), e.toString().c_str());
    }
    return 0;
  }
  BSONObj update;
  try{
    update = fromjson(update_string);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Can't parse update_string '%s'\n Exception: %s",
          update_string.c_str(), e.toString().c_str());
    }
    return 0;
  }

  //actually execute update
  try{
    mongodb_client_->update(collection, query, update);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Error for update %s for query %s\n Exception: %s",
          update_string.c_str(), query_string.c_str(), e.toString().c_str());
    }
    return 0;
  }
  //return success
  return 1;
}

int RobotMemory::remove(std::string query_string, std::string collection)
{
  if(collection == "")
  {
    collection = default_collection_;
  }
  if(debug_)
  {
    logger_->log_info(name_, "Executing Remove %s on collection %s",
        query_string.c_str(), collection);
  }

  //only one query at a time
  MutexLocker lock(mutex_);

  //get query from string
  Query query;
  try{
    query = Query(query_string);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Can't parse query_string '%s'\n Exception: %s",
          query_string.c_str(), e.toString().c_str());
    }
    return 0;
  }

  //actually execute remove
  try{
    mongodb_client_->remove(collection, query);
  } catch (DBException &e) {
    if(debug_)
    {
      logger_->log_error(name_, "Error for query %s\n Exception: %s",
          query_string.c_str(), e.toString().c_str());
    }
    return 0;
  }
  //return success
  return 1;
}

void
RobotMemory::log(Query query, std::string what)
{
  std::string output = what
    + "\nFilter: " + query.getFilter().toString()
    + "\nModifiers: " + query.getModifiers().toString()
    + "\nSort: " + query.getSort().toString()
    + "\nHint: " + query.getHint().toString()
    + "\nReadPref: " + query.getReadPref().toString();

  logger_->log_info(name_, "%s", output.c_str());
}

void
RobotMemory::log(BSONObj obj, std::string what)
{
  std::string output = what
    + "\nObject: " + obj.toString();

  logger_->log_info(name_, "%s", output.c_str());
}

void
RobotMemory::set_fields(BSONObj &obj, std::string what)
{
  BSONObjBuilder b;
  b.appendElements(obj);
  b.appendElements(fromjson(what));
  //override
  obj = b.obj();
}

void
RobotMemory::set_fields(Query &q, std::string what)
{
  BSONObjBuilder b;
  b.appendElements(q.getFilter());
  b.appendElements(fromjson(what));

  //TODO keep other stuff in query
  // + "\nFilter: " + query.getFilter().toString()
  // + "\nModifiers: " + query.getModifiers().toString()
  // + "\nSort: " + query.getSort().toString()
  // + "\nHint: " + query.getHint().toString()
  // + "\nReadPref: " + query.getReadPref().toString();

  //override
  q = Query(b.obj());
}

void
RobotMemory::remove_field(Query &q, std::string what)
{
  BSONObjBuilder b;
  b.appendElements(q.getFilter().removeField(what));

  //TODO keep other stuff in query
  // + "\nFilter: " + query.getFilter().toString()
  // + "\nModifiers: " + query.getModifiers().toString()
  // + "\nSort: " + query.getSort().toString()
  // + "\nHint: " + query.getHint().toString()
  // + "\nReadPref: " + query.getReadPref().toString();

  //override
  q = Query(b.obj());
}

