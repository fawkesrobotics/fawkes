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
  log("Started RobotMemory");
  default_collection_ = "robmem.test";
  try {
    default_collection_ = config_->get_string("/plugins/robot-memory/default-collection");
  } catch (Exception &e) {}
  try {
    debug_ = config_->get_bool("/plugins/robot-memory/more-debug-output");
  } catch (Exception &e) {}
  database_name_ = "mobmem";
  try {
    database_name_ = config_->get_string("/plugins/robot-memory/database");
  } catch (Exception &e) {}

  //init blackboard interface
  rm_if_ = blackboard_->open_for_writing<RobotMemoryInterface>(config_->get_string("/plugins/robot-memory/interface-name").c_str());
  rm_if_->set_error("");
  rm_if_->set_result("");
  rm_if_->write();

  log_deb("Initialized RobotMemory");
}

QResCursor RobotMemory::query(std::string query_string, std::string collection)
{
  if(collection == "")
  {
    collection = default_collection_;
  }
  log_deb(std::string("Executing Query "+ query_string+" on collection "+collection));

  //only one query at a time
  MutexLocker lock(mutex_);

  //get query from string
  Query query;
  try{
    query = Query(query_string);
  } catch (DBException &e) {
    std::string error = "Can't parse query_string "
        + query_string + "\n Exception: " + e.toString();
    log_deb(error, "error");
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
    std::string error = std::string("Error for query ")
      + query_string + "\n Exception: " + e.toString();
    log(error, "error");
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

  log_deb(std::string("Executing Query "+ insert_string + " on collection " + collection));

  //only one query at a time
  MutexLocker lock(mutex_);

  //get query from string
  BSONObj obj;
  try{
    obj = fromjson(insert_string);
  } catch (DBException &e) {
    std::string error = "Can't parse insert_string "
        + insert_string + "\n Exception: " + e.toString();
    log_deb(error, "error");
    return 0;
  }

  set_fields(obj, "{type: \"test\"}");

  //actually execute insert
  try{
    mongodb_client_->insert(collection, obj);
  } catch (DBException &e) {
    std::string error = "Error for insert " + insert_string
        + "\n Exception: " + e.toString();
    log_deb(error, "error");
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
  log_deb(std::string("Executing Update "+update_string+" for query "+query_string+" on collection "+ collection));

  //only one query at a time
  MutexLocker lock(mutex_);

  //get query from string
  Query query;
  try{
    query = Query(query_string);
  } catch (DBException &e) {
    std::string error = "Can't parse query_string " + query_string
        + "\n Exception: " + e.toString();
    log_deb(error, "error");
    return 0;
  }
  BSONObj update;
  try{
    update = fromjson(update_string);
  } catch (DBException &e) {
    log_deb(std::string("Can't parse update_string '"+update_string+"'\n Exception: "+e.toString()),"error");
    return 0;
  }

  //actually execute update
  try{
    mongodb_client_->update(collection, query, update);
  } catch (DBException &e) {
    log_deb(std::string("Error for update "+update_string+" for query "+query_string+"\n Exception: "+e.toString()), "error");
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
  log_deb(std::string("Executing Remove "+query_string+" on collection "+collection));

  //only one query at a time
  MutexLocker lock(mutex_);

  //get query from string
  Query query;
  try{
    query = Query(query_string);
  } catch (DBException &e) {
    log_deb(std::string("Can't parse query_string '"+query_string+"'\n Exception: "+e.toString()), "error");
    return 0;
  }

  //actually execute remove
  try{
    mongodb_client_->remove(collection, query);
  } catch (DBException &e) {
    log_deb(std::string("Error for query "+query_string+"\n Exception: "+e.toString()), "error");
    return 0;
  }
  //return success
  return 1;
}

int RobotMemory::drop_collection(std::string collection)
{
  log_deb("Clearing whole robot memory");
  return remove("{}", collection);
}

int RobotMemory::clear_memory()
{
  log_deb("Clearing whole robot memory");
  mongodb_client_->dropDatabase(database_name_);
  return 1;
}

void
RobotMemory::log(std::string what, std::string info)
{
  if(!info.compare("error"))
      logger_->log_error(name_, "%s", what.c_str());
  else if(!info.compare("warn"))
    logger_->log_warn(name_, "%s", what.c_str());
  else if(!info.compare("debug"))
    logger_->log_debug(name_, "%s", what.c_str());
  else
    logger_->log_info(name_, "%s", what.c_str());
}

void
RobotMemory::log_deb(std::string what, std::string level)
{
  if(debug_)
    log(what, level);
}

void
RobotMemory::log_deb(Query query, std::string what, std::string level)
{
  if(debug_)
    log(query, what, level);
}

void
RobotMemory::log(Query query, std::string what, std::string level)
{
  std::string output = what
    + "\nFilter: " + query.getFilter().toString()
    + "\nModifiers: " + query.getModifiers().toString()
    + "\nSort: " + query.getSort().toString()
    + "\nHint: " + query.getHint().toString()
    + "\nReadPref: " + query.getReadPref().toString();
  log(output, level);
}

void
RobotMemory::log_deb(BSONObj obj, std::string what, std::string level)
{
  log(obj, what, level);
}

void
RobotMemory::log(BSONObj obj, std::string what, std::string level)
{
  std::string output = what
    + "\nObject: " + obj.toString();
  log(output, level);
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

