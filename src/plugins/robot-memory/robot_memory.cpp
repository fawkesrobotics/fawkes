/***************************************************************************
 *  robot_memory.cpp - Class for storing and querying information in the RobotMemory
 *    
 *  Created: Aug 23, 2016 1:34:32 PM 2016
 *  Copyright  2016  Frederik Zwilling
 *             2017 Tim Niemueller [www.niemueller.de]
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
#include <interfaces/RobotMemoryInterface.h>
#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <utils/system/hostinfo.h>

#include <string>
#include <chrono>
#include <thread>

// from MongoDB
#include <mongo/client/dbclient.h>

using namespace mongo;
using namespace fawkes;

/** @class RobotMemory "robot_memory.h"
 * Access to the robot memory based on mongodb.
 * Using this class, you can query/insert/remove/update information in
 * the robot memory.  Furthermore, you can register trigger to get
 * notified when something was changed in the robot memory matching
 * your query and you can access computables, which are on demand
 * computed information, by registering the computables and then
 * querying as if the information would already be in the database.
 * @author Frederik Zwilling
 */

/**
 * Robot Memory Constructor with objects of the thread
 * @param config Fawkes config
 * @param logger Fawkes logger
 * @param clock Fawkes clock
 * @param mongo_connection_manager MongoDBConnCreator to create client connections to the shared and local db
 * @param blackboard Fawkes blackboard
 */
RobotMemory::RobotMemory(fawkes::Configuration* config, fawkes::Logger* logger,
   fawkes::Clock* clock, fawkes::MongoDBConnCreator* mongo_connection_manager,
   fawkes::BlackBoard* blackboard)
{
  mutex_ = new Mutex();
  config_ = config;
  logger_ = logger;
  clock_ = clock;
  mongo_connection_manager_ = mongo_connection_manager;
  blackboard_ = blackboard;
  debug_ = false;
}

RobotMemory::~RobotMemory()
{
  mongo_connection_manager_->delete_client(mongodb_client_local_);
  mongo_connection_manager_->delete_client(mongodb_client_distributed_);
  delete mutex_;
  delete trigger_manager_;
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
  database_name_ = "robmem";
  try {
    database_name_ = config_->get_string("/plugins/robot-memory/database");
  } catch (Exception &e) {}
  distributed_dbs_ = config_->get_strings("/plugins/robot-memory/distributed-db-names");
  cfg_startup_grace_period_ = 10;
  try {
	  cfg_startup_grace_period_ = config_->get_uint("/plugins/robot-memory/startup-grace-period");
  } catch (Exception &e) {} // ignored, use default

  cfg_coord_database_ = config_->get_string("/plugins/robot-memory/coordination/database");
  cfg_coord_mutex_collection_ = config_->get_string("/plugins/robot-memory/coordination/mutex-collection");
  cfg_coord_mutex_collection_ = cfg_coord_database_ + "." + cfg_coord_mutex_collection_;

  using namespace std::chrono_literals;
  
  //initiate mongodb connections:
  log("Connect to local mongod");
  unsigned int startup_tries = 0;
  for (; startup_tries < cfg_startup_grace_period_ * 2; ++startup_tries) {
	  try {
		  mongodb_client_local_ = mongo_connection_manager_->create_client("robot-memory-local");
		  break;
	  } catch (fawkes::Exception &e) {
		  logger_->log_info(name_, "Waiting");
		  std::this_thread::sleep_for(500ms);
	  }
  }

  if (config_->exists("/plugins/mongodb/clients/robot-memory-distributed/enabled") &&
      config_->get_bool("/plugins/mongodb/clients/robot-memory-distributed/enabled"))
  {
	  distributed_ = true;
    log("Connect to distributed mongod");
    for (; startup_tries < cfg_startup_grace_period_ * 2; ++startup_tries) {
	    try {
		    mongodb_client_distributed_ = mongo_connection_manager_->create_client("robot-memory-distributed");
		    break;
	    } catch (fawkes::Exception &e) {
		    logger_->log_info(name_, "Waiting");
		    std::this_thread::sleep_for(500ms);
	    }
    }
  }

  //init blackboard interface
  rm_if_ = blackboard_->open_for_writing<RobotMemoryInterface>(config_->get_string("/plugins/robot-memory/interface-name").c_str());
  rm_if_->set_error("");
  rm_if_->set_result("");
  rm_if_->write();

  //Setup event trigger and computables manager
  trigger_manager_ = new EventTriggerManager(logger_, config_, mongo_connection_manager_);
  computables_manager_ = new ComputablesManager(logger_, config_, this, clock_);

  log_deb("Initialized RobotMemory");
}

void RobotMemory::loop()
{
  trigger_manager_->check_events();
  computables_manager_->cleanup_computed_docs();
}

/**
 * Query information from the robot memory.
 * @param query The query returned documents have to match (essentially a BSONObj)
 * @param collection The database and collection to query as string (e.g. robmem.worldmodel)
 * @return Cursor to get the documents from, NULL for invalid query
 */
QResCursor RobotMemory::query(Query query, std::string collection)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);
  log_deb(std::string("Executing Query "+ query.toString() +" on collection "+collection));

  //check if computation on demand is necessary and execute Computables
  computables_manager_->check_and_compute(query, collection);

  //lock (mongo_client not thread safe)
  MutexLocker lock(mutex_);

  //set read preference of query to nearest to read from the local replica set member first
  query.readPref(ReadPreference_Nearest, BSONArray());

  //actually execute query
  QResCursor cursor;
  try{
    cursor = mongodb_client->query(collection, query);
  } catch (DBException &e) {
    std::string error = std::string("Error for query ")
      + query.toString() + "\n Exception: " + e.toString();
    log(error, "error");
    return NULL;
  }
  return cursor;
}

/**
 * Aggregation call on the robot memory.
 * @param pipeline Series of commands defining the aggregation
 * @param collection The database and collection to query as string (e.g. robmem.worldmodel)
 * @return Result object
 */
mongo::BSONObj
RobotMemory::aggregate(std::vector<mongo::BSONObj> pipeline, std::string collection)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);
  log_deb(std::string("Executing Aggregation on collection "+collection));

  //TODO: check if computation on demand is necessary and execute Computables
  // that might be complicated because you need to build a query to check against from the fields mentioned in the different parts of the pipeline
  // A possible solution might be forcing the user to define the $match oject seperately and using it as query to check computables

  //lock (mongo_client not thread safe)
  MutexLocker lock(mutex_);

  //actually execute aggregation as command (in more modern mongo-cxx versions there should be an easier way with a proper aggregate function)
  BSONObj res;
  //get db and collection name
  size_t point_pos = collection.find(".");
  if(point_pos == collection.npos)
  {
    logger_->log_error(name_, "Collection %s needs to start with 'dbname.'", collection.c_str());
    return fromjson("{}");
  }
  std::string db = collection.substr(0, point_pos);
  std::string col = collection.substr(point_pos+1);
  try{
    mongodb_client->runCommand(db, BSON("aggregate" << col  << "pipeline" << pipeline), res);
  } catch (DBException &e) {
    std::string error = std::string("Error for aggregation ")
      + "\n Exception: " + e.toString();
    log(error, "error");
    return fromjson("{}");
  }
  return res;
}

/**
 * Inserts a document into the robot memory
 * @param obj The document as BSONObj
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int RobotMemory::insert(mongo::BSONObj obj, std::string collection)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);

  log_deb(std::string("Inserting "+ obj.toString() + " into collection " + collection));

  //lock (mongo_client not thread safe)
  MutexLocker lock(mutex_);

  //actually execute insert
  try{
    mongodb_client->insert(collection, obj);
  } catch (DBException &e) {
    std::string error = "Error for insert " + obj.toString()
        + "\n Exception: " + e.toString();
    log_deb(error, "error");
    return 0;
  }
  //return success
  return 1;
}

/** Create an index on a collection.
 * @param obj The keys document as BSONObj
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @param unique true to create unique index
 * @return 1: Success 0: Error
 */
int
RobotMemory::create_index(mongo::BSONObj obj, std::string collection, bool unique)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);

  log_deb(std::string("Creating index "+ obj.toString() + " on collection " + collection));

  //lock (mongo_client not thread safe)
  MutexLocker lock(mutex_);

  //actually execute insert
  try{
	  mongodb_client->createIndex(collection, mongo::IndexSpec().addKeys(obj).unique(unique));
  } catch (DBException &e) {
	  std::string error = "Error when creating index " + obj.toString()
		  + "\n Exception: " + e.toString();
	  log_deb(error, "error");
	  return 0;
  }
  //return success
  return 1;
}


/**
 * Inserts all document of a vector into the robot memory
 * @param v_obj The vector of BSONObj document
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int RobotMemory::insert(std::vector<mongo::BSONObj> v_obj, std::string collection)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);

  std::string insert_string = "[";
  for(BSONObj obj : v_obj)
  {
    insert_string += obj.toString() + ",\n";
  }
  insert_string += "]";

  log_deb(std::string("Inserting vector of documents " + insert_string+  " into collection " + collection));

  //lock (mongo_client not thread safe)
  MutexLocker lock(mutex_);

  //actually execute insert
  try{
    mongodb_client->insert(collection, v_obj);
  } catch (DBException &e) {
    std::string error = "Error for insert " + insert_string
        + "\n Exception: " + e.toString();
    log_deb(error, "error");
    return 0;
  }
  //return success
  return 1;
}

/**
 * Inserts a document into the robot memory
 * @param obj_str The document as json string
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int RobotMemory::insert(std::string obj_str, std::string collection)
{
  return insert(fromjson(obj_str), collection);
}

/**
 * Updates documents in the robot memory
 * @param query The query defining which documents to update
 * @param update What to change in these documents
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @param upsert Should the update document be inserted if the query returns no documents?
 * @return 1: Success 0: Error
 */
int RobotMemory::update(mongo::Query query, mongo::BSONObj update, std::string collection, bool upsert)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);
  log_deb(std::string("Executing Update "+update.toString()+" for query "+query.toString()+" on collection "+ collection));

  //lock (mongo_client not thread safe)
  MutexLocker lock(mutex_);

  //actually execute update
  try{
    mongodb_client->update(collection, query, update, upsert);
  } catch (DBException &e) {
    log_deb(std::string("Error for update "+update.toString()+" for query "+query.toString()+"\n Exception: "+e.toString()), "error");
    return 0;
  }
  //return success
  return 1;
}

/**
 * Updates documents in the robot memory
 * @param query The query defining which documents to update
 * @param update_str What to change in these documents as json string
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @param upsert Should the update document be inserted if the query returns no documents?
 * @return 1: Success 0: Error
 */
int RobotMemory::update(mongo::Query query, std::string update_str, std::string collection, bool upsert)
{
  return update(query, fromjson(update_str), collection, upsert);
}


/** Atomically update and retrieve document.
 * @param filter The filter defining the document to update.
 * If multiple match takes the first one.
 * @param update Update statement. May only contain update operators!
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @param upsert Should the update document be inserted if the query returns no documents?
 * @param return_new return the document before (false) or after (true) the update has been applied?
 * @return document, depending on @p return_new either before or after the udpate has been applied.
 */
mongo::BSONObj
RobotMemory::find_one_and_update(const mongo::BSONObj& filter, const mongo::BSONObj& update,
                                 std::string collection, bool upsert, bool return_new)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);

  log_deb(std::string("Executing findOneAndUpdate "+update.toString()+
                      " for filter "+filter.toString()+" on collection "+ collection));

  MutexLocker lock(mutex_);

  try{
	  return mongodb_client->findAndModify(collection, filter, update, upsert, return_new);
  } catch (DBException &e) {
	  std::string error = "Error for update "+update.toString()+" for query "+
		  filter.toString()+"\n Exception: "+e.toString();
	  log_deb(error, "error");
    BSONObjBuilder b;
    b.append("error", error);
    return b.obj();
  }
}

/**
 * Remove documents from the robot memory
 * @param query Which documents to remove
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int RobotMemory::remove(mongo::Query query, std::string collection)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);
  log_deb(std::string("Executing Remove "+query.toString()+" on collection "+collection));

  //lock (mongo_client not thread safe)
  MutexLocker lock(mutex_);

  //actually execute remove
  try{
    mongodb_client->remove(collection, query);
  } catch (DBException &e) {
    log_deb(std::string("Error for query "+query.toString()+"\n Exception: "+e.toString()), "error");
    return 0;
  }
  //return success
  return 1;
}

/**
 * Performs a MapReduce operation on the robot memory (https://docs.mongodb.com/manual/core/map-reduce/)
 * @param query Which documents to use for the map step
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @param js_map_fun Map function in JavaScript as string
 * @param js_reduce_fun Reduce function in JavaScript as string
 * @return BSONObj containing the result
 */
mongo::BSONObj RobotMemory::mapreduce(mongo::Query query, std::string collection, std::string js_map_fun, std::string js_reduce_fun)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);
  MutexLocker lock(mutex_);
  log_deb(std::string("Executing MapReduce "+query.toString()+" on collection "+collection+
    " map: " + js_map_fun + " reduce: " + js_reduce_fun));
  return mongodb_client->mapreduce(collection, js_map_fun, js_reduce_fun, query);
}

/**
 * Performs an aggregation operation on the robot memory (https://docs.mongodb.com/v3.2/reference/method/db.collection.aggregate/)
 * @param pipeline A sequence of data aggregation operations or stages. See the https://docs.mongodb.com/v3.2/reference/operator/aggregation-pipeline/ for details
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @return Cursor to get the documents from, NULL for invalid pipeline
 */
QResCursor RobotMemory::aggregate(mongo::BSONObj pipeline, std::string collection)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);
  MutexLocker lock(mutex_);
  log_deb(std::string("Executing Aggregation pipeline: "+pipeline.toString() +" on collection "+collection));

  QResCursor cursor;
  try{
    cursor = mongodb_client->aggregate(collection, pipeline);
  } catch (DBException &e) {
    std::string error = std::string("Error for query ")
      + pipeline.toString() + "\n Exception: " + e.toString();
    log(error, "error");
    return NULL;
  }
  return cursor;
}

/**
 * Drop (= remove) a whole collection and all documents inside it
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int RobotMemory::drop_collection(std::string collection)
{
  check_collection_name(collection);
  mongo::DBClientBase* mongodb_client = get_mongodb_client(collection);
  MutexLocker lock(mutex_);
  log_deb("Dropping collection " + collection);
  return mongodb_client->dropCollection(collection);
}

/**
 * Remove the whole database of the robot memory and all documents inside
 * @return 1: Success 0: Error
 */
int RobotMemory::clear_memory()
{
  //lock (mongo_client not thread safe)
  MutexLocker lock(mutex_);

  log_deb("Clearing whole robot memory");
  mongodb_client_local_->dropDatabase(database_name_);
  return 1;
}

/**
 * Restore a previously dumped collection from a directory
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @param directory Directory of the dump
 * @return 1: Success 0: Error
 */
int RobotMemory::restore_collection(std::string collection, std::string directory)
{
  check_collection_name(collection);
  drop_collection(collection);

  //lock (mongo_client not thread safe)
   MutexLocker lock(mutex_);

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
  std::string command = "/usr/bin/mongorestore --dir " + path
      + " --host=127.0.0.1 --quiet";
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

/**
 * Dump (= save) a collection to the filesystem to restore it later
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @param directory Directory to dump the collection to
 * @return 1: Success 0: Error
 */
int RobotMemory::dump_collection(std::string collection, std::string directory)
{
  check_collection_name(collection);

  //lock (mongo_client not thread safe)
   MutexLocker lock(mutex_);

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
    + " --collection=" + split[1] + " --host=127.0.0.1 --quiet";
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
RobotMemory::log_deb(mongo::Query query, std::string what, std::string level)
{
  if(debug_)
    log(query, what, level);
}

void
RobotMemory::log(mongo::Query query, std::string what, std::string level)
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
RobotMemory::log_deb(mongo::BSONObj obj, std::string what, std::string level)
{
  log(obj, what, level);
}

void
RobotMemory::log(mongo::BSONObj obj, std::string what, std::string level)
{
  std::string output = what
    + "\nObject: " + obj.toString();
  log(output, level);
}

void
RobotMemory::set_fields(mongo::BSONObj &obj, std::string what)
{
  BSONObjBuilder b;
  b.appendElements(obj);
  b.appendElements(fromjson(what));
  //override
  obj = b.obj();
}

void
RobotMemory::set_fields(mongo::Query &q, std::string what)
{
  BSONObjBuilder b;
  b.appendElements(q.getFilter());
  b.appendElements(fromjson(what));

  //the following is not yet kept in the query:
  // + "\nFilter: " + query.getFilter().toString()
  // + "\nModifiers: " + query.getModifiers().toString()
  // + "\nSort: " + query.getSort().toString()
  // + "\nHint: " + query.getHint().toString()
  // + "\nReadPref: " + query.getReadPref().toString();

  //override
  q = Query(b.obj());
}

void
RobotMemory::remove_field(mongo::Query &q, std::string what)
{
  BSONObjBuilder b;
  b.appendElements(q.getFilter().removeField(what));

  //the following is not yet kept in the query:
  // + "\nFilter: " + query.getFilter().toString()
  // + "\nModifiers: " + query.getModifiers().toString()
  // + "\nSort: " + query.getSort().toString()
  // + "\nHint: " + query.getHint().toString()
  // + "\nReadPref: " + query.getReadPref().toString();

  //override
  q = Query(b.obj());
}

/**
 * Check if collection name is valid and correct it if necessary
 */
void
RobotMemory::check_collection_name(std::string &collection)
{
  if(collection == "")
  {
      collection = default_collection_;
  }
  if(database_name_ != "robmem" && collection.find("robmem.") == 0)
  {
    //change used database name (e.g. for the case of multiple simulated dababases)
    collection.replace(0, 6, database_name_);
  }
}

/**
 * Get the mongodb client associated with the collection (eighter the local or distributed one)
 */
mongo::DBClientBase*
RobotMemory::get_mongodb_client(std::string &collection)
{
  if(!distributed_)
  {
      return mongodb_client_local_;
  }
  //get db name of collection
  size_t point_pos = collection.find(".");
  if(point_pos == collection.npos)
  {
    logger_->log_error(name_, "Collection %s needs to start with 'dbname.'", collection.c_str());
    return mongodb_client_local_;
  }
  std::string db = collection.substr(0, point_pos);
  if(std::find(distributed_dbs_.begin(), distributed_dbs_.end(), db) != distributed_dbs_.end())
  {
    return mongodb_client_distributed_;
  }
  return mongodb_client_local_;
}

/**
 * Remove a previously registered trigger
 * @param trigger Pointer to the trigger to remove
 */
void RobotMemory::remove_trigger(EventTrigger* trigger)
{
  trigger_manager_->remove_trigger(trigger);
}

/**
 * Remove previously registered computable
 * @param computable The computable to remove
 */
void RobotMemory::remove_computable(Computable* computable)
{
  computables_manager_->remove_computable(computable);
}

/** Explicitly create a mutex.
 * This is an optional step, a mutex is also created automatically when trying
 * to acquire the lock for the first time. Adding it explicitly may increase
 * visibility, e.g., in the database. Use it for mutexes which are locked
 * only very infrequently.
 * @param name mutex name
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_create(const std::string& name)
{
	mongo::DBClientInterface *client =
		distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;
	mongo::BSONObjBuilder insert_doc;
	insert_doc.append("$currentDate", BSON("lock-time" << true));
	insert_doc.append("_id", name);
	insert_doc.append("locked", false);
	try {
		client->insert(cfg_coord_mutex_collection_, insert_doc.obj(),
		               0, &mongo::WriteConcern::majority);
		return true;
	} catch (mongo::DBException &e) {
		logger_->log_info(name_, "Failed to create mutex %s: %s", name.c_str(), e.what());
		return false;
	}
}

/** Destroy a mutex.
 * The mutex is erased from the database. This is done disregarding it's current
 * lock state.
 * @param name mutex name
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_destroy(const std::string& name)
{
	mongo::DBClientInterface *client =
		distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;
	mongo::BSONObj destroy_doc{BSON("_id" << name)};
	try {
		client->remove(cfg_coord_mutex_collection_, destroy_doc,
		               true, &mongo::WriteConcern::majority);
		return true;
	} catch (mongo::DBException &e) {
		logger_->log_info(name_, "Failed to destroy mutex %s: %s", name.c_str(), e.what());
		return false;
	}
}

/** Try to acquire a lock for a mutex.
 * This will access the database and atomically find and update (or
 * insert) a mutex lock. If the mutex has not been created it is added
 * automatically. If the lock cannot be acquired the function also
 * returns immediately. There is no blocked waiting for the lock.
 * @param name mutex name
 * @param identity string to set as lock-holder
 * @param force true to force acquisition of the lock, i.e., even if
 * the lock has already been acquired take ownership (steal the lock).
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_try_lock(const std::string& name,
                            std::string identity, bool force)
{
	mongo::DBClientBase *client =
		distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	if (identity.empty()) {
		HostInfo host_info;
		identity = host_info.name();
	}

	// here we can add an $or to implement lock timeouts
	mongo::BSONObjBuilder filter_doc;
	filter_doc.append("_id", name);
	if (! force) {
		filter_doc.append("locked", false);
	}

	mongo::BSONObjBuilder update_doc;
	update_doc.append("$currentDate", BSON("lock-time" << true));
	mongo::BSONObjBuilder update_set;
	update_set.append("locked", true);
	update_set.append("locked-by", identity);
	update_doc.append("$set", update_set.obj());

	try {
		BSONObj new_doc =
			client->findAndModify(cfg_coord_mutex_collection_,
			                      filter_doc.obj(), update_doc.obj(),
			                      /* upsert */ true, /* return new */ true,
			                      /* sort */ BSONObj(), /* fields */ BSONObj(),
			                      &mongo::WriteConcern::majority);

		return (new_doc.getField("locked-by").String() == identity &&
		        new_doc.getField("locked").Bool());

	} catch (mongo::OperationException &e) {
		//if (e.obj()["code"].numberInt() != 11000) {
		// 11000: Duplicate key exception, occurs if we do not become leader, all fine
		return false;
	}
}

/** Try to acquire a lock for a mutex.
 * This will access the database and atomically find and update (or
 * insert) a mutex lock. If the mutex has not been created it is added
 * automatically. If the lock cannot be acquired the function also
 * returns immediately. There is no blocked waiting for the lock.
 * @param name mutex name
 * @param force true to force acquisition of the lock, i.e., even if
 * the lock has already been acquired take ownership (steal the lock).
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_try_lock(const std::string& name, bool force)
{
	return mutex_try_lock(name, "", force);
}

/** Release lock on mutex.
 * @param name mutex name
 * @param identity string to set as lock-holder
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_unlock(const std::string& name,
                          std::string identity)
{
	mongo::DBClientBase *client =
		distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	if (identity.empty()) {
		HostInfo host_info;
		identity = host_info.name();
	}

	// here we can add an $or to implement lock timeouts
	mongo::BSONObj filter_doc{BSON("_id" << name << "locked-by" << identity)};

	mongo::BSONObj update_doc{BSON("$set" << BSON("locked" << false) <<
	                               "$unset" << BSON("locked-by" << true <<
	                                                "lock-time" << true))};

	try {
		BSONObj new_doc =
			client->findAndModify(cfg_coord_mutex_collection_,
			                      filter_doc, update_doc,
			                      /* upsert */ true, /* return new */ true,
			                      /* sort */ BSONObj(), /* fields */ BSONObj(),
			                      &mongo::WriteConcern::majority);

		return true;
	} catch (mongo::OperationException &e) {
		return false;
	}
}


/** Renew a mutex.
 * Renewing means updating the lock timestamp to the current time to
 * avoid expiration. Note that the lock must currently be held by
 * the given identity.
 * @param name mutex name
 * @param identity string to set as lock-holder (defaults to hostname
 * if empty)
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_renew_lock(const std::string& name,
                              std::string identity)
{
	mongo::DBClientBase *client =
		distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	if (identity.empty()) {
		HostInfo host_info;
		identity = host_info.name();
	}

	// here we can add an $or to implement lock timeouts
	mongo::BSONObj filter_doc{BSON("_id" << name <<
	                               "locked" << true <<
	                               "locked-by" << identity)};

	// we set all data, even the data which is not actually modified, to
	// make it easier to process the update in triggers.
	mongo::BSONObjBuilder update_doc;
	update_doc.append("$currentDate", BSON("lock-time" << true));
	mongo::BSONObjBuilder update_set;
	update_set.append("locked", true);
	update_set.append("locked-by", identity);
	update_doc.append("$set", update_set.obj());

	try {
		BSONObj new_doc =
			client->findAndModify(cfg_coord_mutex_collection_,
			                      filter_doc, update_doc.obj(),
			                      /* upsert */ false, /* return new */ true,
			                      /* sort */ BSONObj(), /* fields */ BSONObj(),
			                      &mongo::WriteConcern::majority);

		return true;
	} catch (mongo::OperationException &e) {
		logger_->log_warn(name_, "Renewing lock on mutex %s failed: %s",
		                  name.c_str(), e.what());
		return false;
	}
}


/** Setup time-to-live index for mutexes.
 * Setting up a time-to-live index for mutexes enables automatic
 * expiration through the database. Note, however, that the documents
 * are expired only every 60 seconds. This has two consequences:
 * - max_age_sec lower than 60 seconds cannot be achieved
 * - locks may be held for up to just below 60 seconds longer than
 *   configured, i.e., if the mutex had not yet expired when the
 *   background tasks runs.
 * @param max_age_sec maximum age of locks in seconds
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_setup_ttl(float max_age_sec)
{
  MutexLocker lock(mutex_);

	mongo::DBClientBase *client =
		distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	BSONObj keys = BSON("lock-time" << true);

	try {
		client->createIndex(cfg_coord_mutex_collection_, mongo::IndexSpec()
		                    .addKeys(keys)
		                    .expireAfterSeconds(max_age_sec));
  } catch (DBException &e) {
		logger_->log_warn(name_, "Creating TTL index failed: %s", e.what());
	  return false;
  }
  return true;
}

/** Expire old locks on mutexes.
 * This will update the database and set all mutexes to unlocked for
 * which the lock-time is older than the given maximum age.
 * @param max_age_sec maximum age of locks in seconds
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_expire_locks(float max_age_sec)
{
	mongo::DBClientBase *client =
		distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	using std::chrono::milliseconds;
	using std::chrono::high_resolution_clock;
	using std::chrono::time_point;
	using std::chrono::time_point_cast;

	auto max_age_ms = milliseconds(static_cast<unsigned long int>(std::floor(max_age_sec*1000)));
	time_point<high_resolution_clock, milliseconds> expire_before =
		time_point_cast<milliseconds>(high_resolution_clock::now()) - max_age_ms;
	mongo::Date_t	expire_before_mdb(expire_before.time_since_epoch().count());

	// here we can add an $or to implement lock timeouts
	mongo::BSONObj filter_doc{BSON("locked" << true <<
	                               "lock-time" << mongo::LT << expire_before_mdb)};

	try {
		client->remove(cfg_coord_mutex_collection_, filter_doc,
		               true, &mongo::WriteConcern::majority);

		return true;
	} catch (mongo::OperationException &e) {
		return false;
	}
}
