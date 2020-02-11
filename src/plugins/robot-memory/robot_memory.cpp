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
#include <plugins/mongodb/utils.h>
#include <utils/misc/string_conversions.h>
#include <utils/misc/string_split.h>
#include <utils/system/hostinfo.h>
#ifdef USE_TIMETRACKER
#	include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#include <bsoncxx/builder/basic/document.hpp>
#include <chrono>
#include <mongocxx/client.hpp>
#include <mongocxx/exception/operation_exception.hpp>
#include <mongocxx/read_preference.hpp>
#include <string>
#include <thread>

using namespace fawkes;
using namespace mongocxx;
using namespace bsoncxx;

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
RobotMemory::RobotMemory(fawkes::Configuration *     config,
                         fawkes::Logger *            logger,
                         fawkes::Clock *             clock,
                         fawkes::MongoDBConnCreator *mongo_connection_manager,
                         fawkes::BlackBoard *        blackboard)
{
	config_                     = config;
	logger_                     = logger;
	clock_                      = clock;
	mongo_connection_manager_   = mongo_connection_manager;
	blackboard_                 = blackboard;
	mongodb_client_local_       = nullptr;
	mongodb_client_distributed_ = nullptr;
	debug_                      = false;
}

RobotMemory::~RobotMemory()
{
	mongo_connection_manager_->delete_client(mongodb_client_local_);
	mongo_connection_manager_->delete_client(mongodb_client_distributed_);
	delete trigger_manager_;
	blackboard_->close(rm_if_);
#ifdef USE_TIMETRACKER
	delete tt_;
#endif
}

void
RobotMemory::init()
{
	//load config values
	log("Started RobotMemory");
	default_collection_ = "robmem.test";
	try {
		default_collection_ = config_->get_string("/plugins/robot-memory/default-collection");
	} catch (Exception &) {
	}
	try {
		debug_ = config_->get_bool("/plugins/robot-memory/more-debug-output");
	} catch (Exception &) {
	}
	database_name_ = "robmem";
	try {
		database_name_ = config_->get_string("/plugins/robot-memory/database");
	} catch (Exception &) {
	}
	distributed_dbs_          = config_->get_strings("/plugins/robot-memory/distributed-db-names");
	cfg_startup_grace_period_ = 10;
	try {
		cfg_startup_grace_period_ = config_->get_uint("/plugins/robot-memory/startup-grace-period");
	} catch (Exception &) {
	} // ignored, use default

	cfg_coord_database_ = config_->get_string("/plugins/robot-memory/coordination/database");
	cfg_coord_mutex_collection_ =
	  config_->get_string("/plugins/robot-memory/coordination/mutex-collection");

	using namespace std::chrono_literals;

	//initiate mongodb connections:
	log("Connect to local mongod");
	unsigned int startup_tries = 0;
	for (; startup_tries < cfg_startup_grace_period_ * 2; ++startup_tries) {
		// TODO if the last try fails, the client remains uninitialized
		try {
			mongodb_client_local_ = mongo_connection_manager_->create_client("robot-memory-local");
			break;
		} catch (fawkes::Exception &) {
			logger_->log_info(name_, "Waiting for local");
			std::this_thread::sleep_for(500ms);
		}
	}

	if (config_->exists("/plugins/mongodb/clients/robot-memory-distributed/enabled")
	    && config_->get_bool("/plugins/mongodb/clients/robot-memory-distributed/enabled")) {
		distributed_ = true;
		log("Connect to distributed mongod");
		for (startup_tries = 0; startup_tries < cfg_startup_grace_period_ * 2; ++startup_tries) {
			// TODO if the last try fails, the client remains uninitialized
			try {
				mongodb_client_distributed_ =
				  mongo_connection_manager_->create_client("robot-memory-distributed");
				break;
			} catch (fawkes::Exception &) {
				logger_->log_info(name_, "Waiting for distributed");
				std::this_thread::sleep_for(500ms);
			}
		}
	}

	//init blackboard interface
	rm_if_ = blackboard_->open_for_writing<RobotMemoryInterface>(
	  config_->get_string("/plugins/robot-memory/interface-name").c_str());
	rm_if_->set_error("");
	rm_if_->set_result("");
	rm_if_->write();

	//Setup event trigger and computables manager
	trigger_manager_     = new EventTriggerManager(logger_, config_, mongo_connection_manager_);
	computables_manager_ = new ComputablesManager(config_, this);

	log_deb("Initialized RobotMemory");

#ifdef USE_TIMETRACKER
	tt_           = new TimeTracker();
	tt_loopcount_ = 0;
	ttc_events_   = tt_->add_class("RobotMemory Events");
	ttc_cleanup_  = tt_->add_class("RobotMemory Cleanup");
#endif
}

void
RobotMemory::loop()
{
	TIMETRACK_START(ttc_events_);
	trigger_manager_->check_events();
	TIMETRACK_END(ttc_events_);
	TIMETRACK_START(ttc_cleanup_);
	computables_manager_->cleanup_computed_docs();
	TIMETRACK_END(ttc_cleanup_);
#ifdef USE_TIMETRACKER
	if (++tt_loopcount_ % 5 == 0) {
		tt_->print_to_stdout();
	}
#endif
}

/**
 * Query information from the robot memory.
 * @param query The query returned documents have to match (essentially a BSONObj)
 * @param collection_name The database and collection to query as string (e.g. robmem.worldmodel)
 * @param query_options Optional options to use to query the database
 * @return Cursor to get the documents from, NULL for invalid query
 */
cursor
RobotMemory::query(document::view          query,
                   const std::string &     collection_name,
                   mongocxx::options::find query_options)
{
	collection collection = get_collection(collection_name);
	log_deb(std::string("Executing Query " + to_json(query) + " on collection " + collection_name));

	//check if computation on demand is necessary and execute Computables
	computables_manager_->check_and_compute(query, collection_name);

	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);

	//actually execute query
	try {
		return collection.find(query, query_options);
	} catch (mongocxx::operation_exception &e) {
		std::string error =
		  std::string("Error for query ") + to_json(query) + "\n Exception: " + e.what();
		log(error, "error");
		throw;
	}
}

/**
 * Aggregation call on the robot memory.
 * @param pipeline Series of commands defining the aggregation
 * @param collection The database and collection to query as string (e.g. robmem.worldmodel)
 * @return Result object
 */
bsoncxx::document::value
RobotMemory::aggregate(const std::vector<bsoncxx::document::view> &pipeline,
                       const std::string &                         collection)
{
	/*
  client *mongodb_client = get_mongodb_client(collection);
	log_deb(std::string("Executing Aggregation on collection " + collection));

	//TODO: check if computation on demand is necessary and execute Computables
	// that might be complicated because you need to build a query to check against from the fields mentioned in the different parts of the pipeline
	// A possible solution might be forcing the user to define the $match oject seperately and using it as query to check computables

	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);

	//actually execute aggregation as command (in more modern mongo-cxx versions there should be an easier way with a proper aggregate function)
	BSONObj res;
	//get db and collection name
	size_t point_pos = collection.find(".");
	if (point_pos == std::string::npos) {
		logger_->log_error(name_, "Collection %s needs to start with 'dbname.'", collection.c_str());
		return fromjson("{}");
	}
	std::string db  = collection.substr(0, point_pos);
	std::string col = collection.substr(point_pos + 1);
	try {
		mongodb_client->runCommand(db, BSON("aggregate" << col << "pipeline" << pipeline), res);
	} catch (DBException &e) {
		std::string error = std::string("Error for aggregation ") + "\n Exception: " + e.toString();
		log(error, "error");
		return fromjson("{}");
	}
	return res;
  */
	throw Exception("Not implemented");
}

/**
 * Inserts a document into the robot memory
 * @param doc A view of the document to insert
 * @param collection_name The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int
RobotMemory::insert(bsoncxx::document::view doc, const std::string &collection_name)
{
	collection collection = get_collection(collection_name);
	log_deb(std::string("Inserting " + to_json(doc) + " into collection " + collection_name));
	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);
	//actually execute insert
	try {
		collection.insert_one(doc);
	} catch (mongocxx::operation_exception &e) {
		std::string error = "Error for insert " + to_json(doc) + "\n Exception: " + e.what();
		log_deb(error, "error");
		return 0;
	}
	//return success
	return 1;
}

/** Create an index on a collection.
 * @param keys The keys document
 * @param collection_name The database and collection to use as string (e.g. robmem.worldmodel)
 * @param unique true to create unique index
 * @return 1: Success 0: Error
 */
int
RobotMemory::create_index(bsoncxx::document::view keys,
                          const std::string &     collection_name,
                          bool                    unique)
{
	collection collection = get_collection(collection_name);

	log_deb(std::string("Creating index " + to_json(keys) + " on collection " + collection_name));

	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);

	//actually execute insert
	try {
		using namespace bsoncxx::builder::basic;
		collection.create_index(keys, make_document(kvp("unique", unique)));
	} catch (operation_exception &e) {
		std::string error = "Error when creating index " + to_json(keys) + "\n Exception: " + e.what();
		log_deb(error, "error");
		return 0;
	}
	//return success
	return 1;
}

/**
 * Inserts all document of a vector into the robot memory
 * @param docs The vector of BSON documents as views
 * @param collection_name The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int
RobotMemory::insert(std::vector<bsoncxx::document::view> docs, const std::string &collection_name)
{
	collection  collection    = get_collection(collection_name);
	std::string insert_string = "[";
	for (auto &&doc : docs) {
		insert_string += to_json(doc) + ",\n";
	}
	insert_string += "]";

	log_deb(std::string("Inserting vector of documents " + insert_string + " into collection "
	                    + collection_name));

	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);

	//actually execute insert
	try {
		collection.insert_many(docs);
	} catch (operation_exception &e) {
		std::string error = "Error for insert " + insert_string + "\n Exception: " + e.what();
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
int
RobotMemory::insert(const std::string &obj_str, const std::string &collection)
{
	return insert(from_json(obj_str), collection);
}

/**
 * Updates documents in the robot memory
 * @param query The query defining which documents to update
 * @param update What to change in these documents
 * @param collection_name The database and collection to use as string (e.g. robmem.worldmodel)
 * @param upsert Should the update document be inserted if the query returns no documents?
 * @return 1: Success 0: Error
 */
int
RobotMemory::update(const bsoncxx::document::view &query,
                    const bsoncxx::document::view &update,
                    const std::string &            collection_name,
                    bool                           upsert)
{
	collection collection = get_collection(collection_name);
	log_deb(std::string("Executing Update " + to_json(update) + " for query " + to_json(query)
	                    + " on collection " + collection_name));

	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);

	//actually execute update
	try {
		collection.update_many(query,
		                       builder::basic::make_document(
		                         builder::basic::kvp("$set", builder::concatenate(update))),
		                       options::update().upsert(upsert));
	} catch (operation_exception &e) {
		log_deb(std::string("Error for update " + to_json(update) + " for query " + to_json(query)
		                    + "\n Exception: " + e.what()),
		        "error");
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
int
RobotMemory::update(const bsoncxx::document::view &query,
                    const std::string &            update_str,
                    const std::string &            collection,
                    bool                           upsert)
{
	return update(query, from_json(update_str), collection, upsert);
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
document::value
RobotMemory::find_one_and_update(const document::view &filter,
                                 const document::view &update,
                                 const std::string &   collection_name,
                                 bool                  upsert,
                                 bool                  return_new)
{
	collection collection = get_collection(collection_name);

	log_deb(std::string("Executing findOneAndUpdate " + to_json(update) + " for filter "
	                    + to_json(filter) + " on collection " + collection_name));

	MutexLocker lock(mutex_);

	try {
		auto res =
		  collection.find_one_and_update(filter,
		                                 update,
		                                 options::find_one_and_update().upsert(upsert).return_document(
		                                   return_new ? options::return_document::k_after
		                                              : options::return_document::k_before));
		if (res) {
			return *res;
		} else {
			std::string error = "Error for update " + to_json(update) + " for query " + to_json(filter)
			                    + "FindOneAndUpdate unexpectedly did not return a document";
			log_deb(error, "warn");
			return bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("error", error));
		}
	} catch (operation_exception &e) {
		std::string error = "Error for update " + to_json(update) + " for query " + to_json(filter)
		                    + "\n Exception: " + e.what();
		log_deb(error, "error");
		return bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("error", error));
	}
}

/**
 * Remove documents from the robot memory
 * @param query Which documents to remove
 * @param collection_name The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int
RobotMemory::remove(const bsoncxx::document::view &query, const std::string &collection_name)
{
	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);
	collection  collection = get_collection(collection_name);
	log_deb(std::string("Executing Remove " + to_json(query) + " on collection " + collection_name));
	//actually execute remove
	try {
		collection.delete_many(query);
	} catch (operation_exception &e) {
		log_deb(std::string("Error for query " + to_json(query) + "\n Exception: " + e.what()),
		        "error");
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
bsoncxx::document::value
RobotMemory::mapreduce(const bsoncxx::document::view &query,
                       const std::string &            collection,
                       const std::string &            js_map_fun,
                       const std::string &            js_reduce_fun)
{
	throw Exception("Not implemented");
	/*
	mongo::DBClientBase *mongodb_client = get_mongodb_client(collection);
	MutexLocker          lock(mutex_);
	log_deb(std::string("Executing MapReduce " + query.toString() + " on collection " + collection
	                    + " map: " + js_map_fun + " reduce: " + js_reduce_fun));
	return mongodb_client->mapreduce(collection, js_map_fun, js_reduce_fun, query);
  */
}

/**
 * Performs an aggregation operation on the robot memory (https://docs.mongodb.com/v3.2/reference/method/db.collection.aggregate/)
 * @param pipeline A sequence of data aggregation operations or stages. See the https://docs.mongodb.com/v3.2/reference/operator/aggregation-pipeline/ for details
 * @param collection The database and collection to use as string (e.g. robmem.worldmodel)
 * @return Cursor to get the documents from, NULL for invalid pipeline
 */
cursor
RobotMemory::aggregate(bsoncxx::document::view pipeline, const std::string &collection)
{
	throw Exception("Not implemented");
	/**
	mongo::DBClientBase *mongodb_client = get_mongodb_client(collection);
	MutexLocker          lock(mutex_);
	log_deb(std::string("Executing Aggregation pipeline: " + pipeline.toString() + " on collection "
	                    + collection));

	QResCursor cursor;
	try {
		cursor = mongodb_client->aggregate(collection, pipeline);
	} catch (DBException &e) {
		std::string error =
		  std::string("Error for query ") + pipeline.toString() + "\n Exception: " + e.toString();
		log(error, "error");
		return NULL;
	}
	return cursor;
  */
}

/**
 * Drop (= remove) a whole collection and all documents inside it
 * @param collection_name The database and collection to use as string (e.g. robmem.worldmodel)
 * @return 1: Success 0: Error
 */
int
RobotMemory::drop_collection(const std::string &collection_name)
{
	MutexLocker lock(mutex_);
	collection  collection = get_collection(collection_name);
	log_deb("Dropping collection " + collection_name);
	collection.drop();
	return 1;
}

/**
 * Remove the whole database of the robot memory and all documents inside
 * @return 1: Success 0: Error
 */
int
RobotMemory::clear_memory()
{
	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);

	log_deb("Clearing whole robot memory");
	mongodb_client_local_->database(database_name_).drop();
	return 1;
}

/**
 * Restore a previously dumped collection from a directory
 * @param dbcollection The database and collection to use as string (e.g.
 * robmem.worldmodel)
 * @param directory Directory of the dump
 * @param target_dbcollection Optional different database and collection where
 * the dump is restored to.  If not set, the dump will be restored in the
 * previous place
 * @return 1: Success 0: Error
 */
int
RobotMemory::restore_collection(const std::string &dbcollection,
                                const std::string &directory,
                                std::string        target_dbcollection)
{
	if (target_dbcollection == "") {
		target_dbcollection = dbcollection;
	}

	drop_collection(target_dbcollection);

	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);

	auto [db, collection] = split_db_collection_string(dbcollection);
	std::string path =
	  StringConversions::resolve_path(directory) + "/" + db + "/" + collection + ".bson";
	log_deb(std::string("Restore collection " + collection + " from " + path), "warn");

	auto [target_db, target_collection] = split_db_collection_string(target_dbcollection);

	//call mongorestore from folder with initial restores
	std::string command = "/usr/bin/mongorestore --dir " + path + " -d " + target_db + " -c "
	                      + target_collection + " --host=" + get_hostport(dbcollection);
	log_deb(std::string("Restore command: " + command), "warn");
	FILE *bash_output = popen(command.c_str(), "r");

	//check if output is ok
	if (!bash_output) {
		log(std::string("Unable to restore collection" + collection), "error");
		return 0;
	}
	std::string output_string = "";
	char        buffer[100];
	while (!feof(bash_output)) {
		if (fgets(buffer, 100, bash_output) == NULL) {
			break;
		}
		output_string += buffer;
	}
	pclose(bash_output);
	if (output_string.find("Failed") != std::string::npos) {
		log(std::string("Unable to restore collection" + collection), "error");
		log_deb(output_string, "error");
		return 0;
	}
	return 1;
}

/**
 * Dump (= save) a collection to the filesystem to restore it later
 * @param dbcollection The database and collection to use as string (e.g. robmem.worldmodel)
 * @param directory Directory to dump the collection to
 * @return 1: Success 0: Error
 */
int
RobotMemory::dump_collection(const std::string &dbcollection, const std::string &directory)
{
	//lock (mongo_client not thread safe)
	MutexLocker lock(mutex_);

	std::string path = StringConversions::resolve_path(directory);
	log_deb(std::string("Dump collection " + dbcollection + " into " + path), "warn");

	auto [db, collection] = split_db_collection_string(dbcollection);

	std::string command = "/usr/bin/mongodump --out=" + path + " --db=" + db
	                      + " --collection=" + collection + " --forceTableScan"
	                      + " --host=" + get_hostport(dbcollection);
	log(std::string("Dump command: " + command), "info");
	FILE *bash_output = popen(command.c_str(), "r");
	//check if output is ok
	if (!bash_output) {
		log(std::string("Unable to dump collection" + collection), "error");
		return 0;
	}
	std::string output_string = "";
	char        buffer[100];
	while (!feof(bash_output)) {
		if (fgets(buffer, 100, bash_output) == NULL) {
			break;
		}
		output_string += buffer;
	}
	pclose(bash_output);
	if (output_string.find("Failed") != std::string::npos) {
		log(std::string("Unable to dump collection" + collection), "error");
		log_deb(output_string, "error");
		return 0;
	}
	return 1;
}

void
RobotMemory::log(const std::string &what, const std::string &info)
{
	if (!info.compare("error"))
		logger_->log_error(name_, "%s", what.c_str());
	else if (!info.compare("warn"))
		logger_->log_warn(name_, "%s", what.c_str());
	else if (!info.compare("debug"))
		logger_->log_debug(name_, "%s", what.c_str());
	else
		logger_->log_info(name_, "%s", what.c_str());
}

void
RobotMemory::log_deb(const std::string &what, const std::string &level)
{
	if (debug_) {
		log(what, level);
	}
}

void
RobotMemory::log_deb(const bsoncxx::document::view &query,
                     const std::string &            what,
                     const std::string &            level)
{
	if (debug_) {
		log(query, what, level);
	}
}

void
RobotMemory::log(const bsoncxx::document::view &query,
                 const std::string &            what,
                 const std::string &            level)
{
	log(what + " " + to_json(query), level);
}

/** Check if the given database is a distributed database
 * @param dbcollection A database collection name pair of the form <dbname>.<collname>
 * @return true iff the database is distributed database
 */
bool
RobotMemory::is_distributed_database(const std::string &dbcollection)
{
	return std::find(distributed_dbs_.begin(),
	                 distributed_dbs_.end(),
	                 split_db_collection_string(dbcollection).first)
	       != distributed_dbs_.end();
}

std::string
RobotMemory::get_hostport(const std::string &dbcollection)
{
	if (distributed_ && is_distributed_database(dbcollection)) {
		return config_->get_string("/plugins/mongodb/clients/robot-memory-distributed-direct/hostport");
	} else {
		return config_->get_string("/plugins/mongodb/clients/robot-memory-local-direct/hostport");
	}
}

/**
 * Get the mongodb client associated with the collection (eighter the local or distributed one)
 * @param collection The collection name in the form "<dbname>.<collname>"
 * @return A pointer to the client for the database with name <dbname>
 */
client *
RobotMemory::get_mongodb_client(const std::string &collection)
{
	if (!distributed_) {
		return mongodb_client_local_;
	}
	if (is_distributed_database(collection)) {
		return mongodb_client_distributed_;
	} else {
		return mongodb_client_local_;
	}
}

/**
 * Get the collection object referred to by the given string.
 * @param dbcollection The name of the collection in the form <dbname>.<collname>
 * @return The respective collection object
 */

collection
RobotMemory::get_collection(const std::string &dbcollection)
{
	auto    db_coll_pair = split_db_collection_string(dbcollection);
	client *client;
	if (is_distributed_database(dbcollection)) {
		client = mongodb_client_distributed_;
	} else {
		client = mongodb_client_local_;
	}
	return client->database(db_coll_pair.first)[db_coll_pair.second];
}

/**
 * Remove a previously registered trigger
 * @param trigger Pointer to the trigger to remove
 */
void
RobotMemory::remove_trigger(EventTrigger *trigger)
{
	trigger_manager_->remove_trigger(trigger);
}

/**
 * Remove previously registered computable
 * @param computable The computable to remove
 */
void
RobotMemory::remove_computable(Computable *computable)
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
RobotMemory::mutex_create(const std::string &name)
{
	client *client = distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;
	using namespace bsoncxx::builder;
	basic::document insert_doc{};
	insert_doc.append(basic::kvp("$currentDate", [](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("lock-time", true));
	}));
	insert_doc.append(basic::kvp("_id", name));
	insert_doc.append(basic::kvp("locked", false));
	try {
		MutexLocker lock(mutex_);
		collection  collection    = client->database(cfg_coord_database_)[cfg_coord_mutex_collection_];
		auto        write_concern = mongocxx::write_concern();
		write_concern.majority(std::chrono::milliseconds(0));
		collection.insert_one(insert_doc.view(), options::insert().write_concern(write_concern));
		return true;
	} catch (operation_exception &e) {
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
RobotMemory::mutex_destroy(const std::string &name)
{
	client *client = distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;
	using namespace bsoncxx::builder;
	basic::document destroy_doc;
	destroy_doc.append(basic::kvp("_id", name));
	try {
		MutexLocker lock(mutex_);
		collection  collection    = client->database(cfg_coord_database_)[cfg_coord_mutex_collection_];
		auto        write_concern = mongocxx::write_concern();
		write_concern.majority(std::chrono::milliseconds(0));
		collection.delete_one(destroy_doc.view(),
		                      options::delete_options().write_concern(write_concern));
		return true;
	} catch (operation_exception &e) {
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
RobotMemory::mutex_try_lock(const std::string &name, const std::string &identity, bool force)
{
	client *client = distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	std::string locked_by{identity};
	if (identity.empty()) {
		HostInfo host_info;
		locked_by = host_info.name();
	}

	// here we can add an $or to implement lock timeouts
	using namespace bsoncxx::builder;
	basic::document filter_doc;
	filter_doc.append(basic::kvp("_id", name));
	if (!force) {
		filter_doc.append(basic::kvp("locked", false));
	}

	basic::document update_doc;
	update_doc.append(basic::kvp("$currentDate", [](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("lock-time", true));
	}));
	update_doc.append(basic::kvp("$set", [locked_by](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("locked", true));
		subdoc.append(basic::kvp("locked-by", locked_by));
	}));
	try {
		MutexLocker lock(mutex_);
		collection  collection    = client->database(cfg_coord_database_)[cfg_coord_mutex_collection_];
		auto        write_concern = mongocxx::write_concern();
		write_concern.majority(std::chrono::milliseconds(0));
		auto new_doc =
		  collection.find_one_and_update(filter_doc.view(),
		                                 update_doc.view(),
		                                 options::find_one_and_update()
		                                   .upsert(true)
		                                   .return_document(options::return_document::k_after)
		                                   .write_concern(write_concern));

		if (!new_doc) {
			return false;
		}
		auto new_view = new_doc->view();
		return (new_view["locked-by"].get_utf8().value.to_string() == locked_by
		        && new_view["locked"].get_bool());

	} catch (operation_exception &e) {
		logger_->log_error(name_, "Mongo OperationException: %s", e.what());
		try {
			// TODO is this extrac check still needed?
			basic::document check_doc;
			check_doc.append(basic::kvp("_id", name));
			check_doc.append(basic::kvp("locked", true));
			check_doc.append(basic::kvp("locked-by", locked_by));
			MutexLocker lock(mutex_);
			collection  collection = client->database(cfg_coord_database_)[cfg_coord_mutex_collection_];
			auto        res        = collection.find_one(check_doc.view());
			logger_->log_info(name_, "Checking whether mutex was acquired succeeded");
			if (res) {
				logger_->log_warn(name_,
				                  "Exception during try-lock for %s, "
				                  "but mutex was still acquired",
				                  name.c_str());
			} else {
				logger_->log_info(name_,
				                  "Exception during try-lock for %s, "
				                  "and mutex was not acquired",
				                  name.c_str());
			}
			return static_cast<bool>(res);
		} catch (operation_exception &e) {
			logger_->log_error(name_,
			                   "Mongo OperationException while handling "
			                   "the first exception: %s",
			                   e.what());
			return false;
		}
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
RobotMemory::mutex_try_lock(const std::string &name, bool force)
{
	return mutex_try_lock(name, "", force);
}

/** Release lock on mutex.
 * @param name mutex name
 * @param identity string to set as lock-holder
 * @return true if operation was successful, false on failure
 */
bool
RobotMemory::mutex_unlock(const std::string &name, const std::string &identity)
{
	client *client = distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	std::string locked_by{identity};
	if (identity.empty()) {
		HostInfo host_info;
		locked_by = host_info.name();
	}

	using namespace bsoncxx::builder;
	// here we can add an $or to implement lock timeouts
	basic::document filter_doc;
	filter_doc.append(basic::kvp("_id", name));
	filter_doc.append(basic::kvp("locked-by", locked_by));

	basic::document update_doc;
	update_doc.append(basic::kvp("$set", [](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("locked", false));
	}));
	update_doc.append(basic::kvp("$unset", [](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("locked-by", true));
		subdoc.append(basic::kvp("lock-time", true));
	}));

	try {
		MutexLocker lock(mutex_);
		collection  collection    = client->database(cfg_coord_database_)[cfg_coord_mutex_collection_];
		auto        write_concern = mongocxx::write_concern();
		write_concern.majority(std::chrono::milliseconds(0));
		auto new_doc =
		  collection.find_one_and_update(filter_doc.view(),
		                                 update_doc.view(),
		                                 options::find_one_and_update()
		                                   .upsert(true)
		                                   .return_document(options::return_document::k_after)
		                                   .write_concern(write_concern));
		if (!new_doc) {
			return false;
		}
		return new_doc->view()["locked"].get_bool();
	} catch (operation_exception &e) {
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
RobotMemory::mutex_renew_lock(const std::string &name, const std::string &identity)
{
	client *client = distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	std::string locked_by{identity};
	if (identity.empty()) {
		HostInfo host_info;
		locked_by = host_info.name();
	}

	using namespace bsoncxx::builder;
	// here we can add an $or to implement lock timeouts
	basic::document filter_doc;
	filter_doc.append(basic::kvp("_id", name));
	filter_doc.append(basic::kvp("locked", true));
	filter_doc.append(basic::kvp("locked-by", locked_by));

	// we set all data, even the data which is not actually modified, to
	// make it easier to process the update in triggers.
	basic::document update_doc;
	update_doc.append(basic::kvp("$currentDate", [](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("lock-time", true));
	}));
	update_doc.append(basic::kvp("$set", [locked_by](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("locked", true));
		subdoc.append(basic::kvp("locked-by", locked_by));
	}));

	try {
		MutexLocker lock(mutex_);
		collection  collection    = client->database(cfg_coord_database_)[cfg_coord_mutex_collection_];
		auto        write_concern = mongocxx::write_concern();
		write_concern.majority(std::chrono::milliseconds(0));
		auto new_doc =
		  collection.find_one_and_update(filter_doc.view(),
		                                 update_doc.view(),
		                                 options::find_one_and_update()
		                                   .upsert(false)
		                                   .return_document(options::return_document::k_after)
		                                   .write_concern(write_concern));
		return static_cast<bool>(new_doc);
	} catch (operation_exception &e) {
		logger_->log_warn(name_, "Renewing lock on mutex %s failed: %s", name.c_str(), e.what());
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

	client *client = distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	auto keys = builder::basic::make_document(builder::basic::kvp("lock-time", true));

	try {
		collection collection = client->database(cfg_coord_database_)[cfg_coord_mutex_collection_];
		collection.create_index(keys.view(),
		                        builder::basic::make_document(
		                          builder::basic::kvp("expireAfterSeconds", max_age_sec)));
	} catch (operation_exception &e) {
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
	client *client = distributed_ ? mongodb_client_distributed_ : mongodb_client_local_;

	using std::chrono::high_resolution_clock;
	using std::chrono::milliseconds;
	using std::chrono::time_point;
	using std::chrono::time_point_cast;

	auto max_age_ms = milliseconds(static_cast<unsigned long int>(std::floor(max_age_sec * 1000)));
	time_point<high_resolution_clock, milliseconds> expire_before =
	  time_point_cast<milliseconds>(high_resolution_clock::now()) - max_age_ms;
	types::b_date expire_before_mdb(expire_before);

	// here we can add an $or to implement lock timeouts
	using namespace bsoncxx::builder;
	basic::document filter_doc;
	filter_doc.append(basic::kvp("locked", true));
	filter_doc.append(basic::kvp("lock-time", [expire_before_mdb](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("$lt", expire_before_mdb));
	}));

	basic::document update_doc;
	update_doc.append(basic::kvp("$set", [](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("locked", false));
	}));
	update_doc.append(basic::kvp("$unset", [](basic::sub_document subdoc) {
		subdoc.append(basic::kvp("locked-by", true));
		subdoc.append(basic::kvp("lock-time", true));
	}));

	try {
		MutexLocker lock(mutex_);
		collection  collection    = client->database(cfg_coord_database_)[cfg_coord_mutex_collection_];
		auto        write_concern = mongocxx::write_concern();
		write_concern.majority(std::chrono::milliseconds(0));
		collection.update_many(filter_doc.view(),
		                       update_doc.view(),
		                       options::update().write_concern(write_concern));
		return true;
	} catch (operation_exception &e) {
		log(std::string("Failed to expire locks: " + std::string(e.what())), "error");
		return false;
	}
}
