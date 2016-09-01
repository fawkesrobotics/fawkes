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
#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <memory>
#include <string>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>

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

QResCursor RobotMemory::query(Query query, std::string collection)
{
  if(collection == "")
    collection = default_collection_;
  log_deb(std::string("Executing Query "+ query.toString() +" on collection "+collection));

  //only one query at a time
  MutexLocker lock(mutex_);

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
      + query.toString() + "\n Exception: " + e.toString();
    log(error, "error");
    return NULL;
  }
  return cursor;
}

int RobotMemory::insert(BSONObj obj, std::string collection)
{
  if(collection == "")
    collection = default_collection_;

  log_deb(std::string("Executing Query "+ obj.toString() + " on collection " + collection));

  //only one query at a time
  MutexLocker lock(mutex_);

  //actually execute insert
  try{
    mongodb_client_->insert(collection, obj);
  } catch (DBException &e) {
    std::string error = "Error for insert " + obj.toString()
        + "\n Exception: " + e.toString();
    log_deb(error, "error");
    return 0;
  }
  //return success
  return 1;
}

int RobotMemory::insert(std::string obj_str, std::string collection)
{
  return insert(fromjson(obj_str), collection);
}

int RobotMemory::update(Query query, BSONObj update, std::string collection, bool upsert)
{
  if(collection == "")
  {
    collection = default_collection_;
  }
  log_deb(std::string("Executing Update "+update.toString()+" for query "+query.toString()+" on collection "+ collection));

  //only one query at a time
  MutexLocker lock(mutex_);

  //actually execute update
  try{
    mongodb_client_->update(collection, query, update, upsert);
  } catch (DBException &e) {
    log_deb(std::string("Error for update "+update.toString()+" for query "+query.toString()+"\n Exception: "+e.toString()), "error");
    return 0;
  }
  //return success
  return 1;
}

int RobotMemory::update(Query query, std::string update_str, std::string collection, bool upsert)
{
  return update(query, fromjson(update_str), collection, upsert);
}

int RobotMemory::remove(Query query, std::string collection)
{
  if(collection == "")
  {
    collection = default_collection_;
  }
  log_deb(std::string("Executing Remove "+query.toString()+" on collection "+collection));

  //only one query at a time
  MutexLocker lock(mutex_);

  //actually execute remove
  try{
    mongodb_client_->remove(collection, query);
  } catch (DBException &e) {
    log_deb(std::string("Error for query "+query.toString()+"\n Exception: "+e.toString()), "error");
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

int RobotMemory::restore_collection(std::string collection, std::string directory)
{
  drop_collection(collection);

  //resolve path to restore
  if(collection.find(".") == std::string::npos)
  {
    log(std::string("Unable to restore collection" + collection), "error");
    log(std::string("Specify collection like 'db.collection'"), "error");
    return 0;
  }
  std::string path = StringConversions::resolve_path(directory) + "/"
      + collection.replace(collection.find("."),1,"/") + ".bson";
  log_deb(std::string("Restore collection " + collection + " from " + path), "warn");

  //call mongorestore from folder with initial restores
  std::string command = "/usr/bin/mongorestore --dir " + path + " --quiet";
  log_deb(std::string("Restore command: " + command), "warn");
  FILE *bash_output = popen(command.c_str(), "r");

  //check if output is ok
  if(!bash_output)
  {
    log(std::string("Unable to restore collection" + collection), "error");
    return 0;
  }
  std::string output_string = "";
  char buffer[100];
  while (!feof(bash_output) )
  {
    if (fgets(buffer, 100, bash_output) == NULL)
    {
      break;
    }
    output_string += buffer;
  }
  pclose(bash_output);
  if(output_string.find("Failed") != std::string::npos)
  {
    log(std::string("Unable to restore collection" + collection), "error");
    log_deb(output_string, "error");
    return 0;
  }
  return 1;
}

int RobotMemory::dump_collection(std::string collection, std::string directory)
{
  //resolve path to dump to
  if(collection.find(".") == std::string::npos)
  {
    log(std::string("Unable to dump collection" + collection), "error");
    log(std::string("Specify collection like 'db.collection'"), "error");
    return 0;
  }
  std::string path = StringConversions::resolve_path(directory);
  log_deb(std::string("Dump collection " + collection + " into " + path), "warn");

  //call mongorestore from folder with initial restores
  std::vector<std::string> split = str_split(collection, '.');
  std::string command = "/usr/bin/mongodump --out=" + path + " --db=" + split[0]
    + " --collection=" + split[1] + " --quiet";
  log_deb(std::string("Dump command: " + command), "warn");
  FILE *bash_output = popen(command.c_str(), "r");

  //check if output is ok
  if(!bash_output)
  {
    log(std::string("Unable to dump collection" + collection), "error");
    return 0;
  }
  std::string output_string = "";
  char buffer[100];
  while (!feof(bash_output) )
  {
    if (fgets(buffer, 100, bash_output) == NULL)
    {
      break;
    }
    output_string += buffer;
  }
  pclose(bash_output);
  if(output_string.find("Failed") != std::string::npos)
  {
    log(std::string("Unable to dump collection" + collection), "error");
    log_deb(output_string, "error");
    return 0;
  }
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
