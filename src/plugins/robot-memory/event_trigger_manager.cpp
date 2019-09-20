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

#ifdef USE_TIMETRACKER
#	include <utils/time/tracker.h>
#endif
#include <plugins/mongodb/utils.h>
#include <utils/time/tracker_macros.h>

#include <boost/bind.hpp>
#include <bsoncxx/json.hpp>
#include <mongocxx/exception/operation_exception.hpp>
#include <mongocxx/exception/query_exception.hpp>

using namespace fawkes;
using namespace mongocxx;

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
EventTriggerManager::EventTriggerManager(Logger *            logger,
                                         Configuration *     config,
                                         MongoDBConnCreator *mongo_connection_manager)
: cfg_debug_(false)
{
	logger_                   = logger;
	config_                   = config;
	mongo_connection_manager_ = mongo_connection_manager;

	con_local_ = mongo_connection_manager_->create_client("robot-memory-local");
	if (config_->exists("/plugins/mongodb/clients/robot-memory-distributed/enabled")
	    && config_->get_bool("/plugins/mongodb/clients/robot-memory-distributed/enabled")) {
		con_replica_ = mongo_connection_manager_->create_client("robot-memory-distributed");
	}

	// create connections to running mongod instances because only there
	std::string local_db = config_->get_string("/plugins/robot-memory/database");
	dbnames_local_.push_back(local_db);
	dbnames_distributed_ = config_->get_strings("/plugins/robot-memory/distributed-db-names");

	mutex_ = new Mutex();

	try {
		cfg_debug_ = config->get_bool("/plugins/robot-memory/more-debug-output");
	} catch (...) {
	}
#ifdef USE_TIMETRACKER
	tt_                = new fawkes::TimeTracker();
	ttc_trigger_loop_  = tt_->add_class("RM Trigger Trigger Loop");
	ttc_callback_loop_ = tt_->add_class("RM Trigger Callback Loop");
	ttc_callback_      = tt_->add_class("RM Trigger Single Callback");
	ttc_reinit_        = tt_->add_class("RM Trigger Reinit");
#endif
}

EventTriggerManager::~EventTriggerManager()
{
	for (EventTrigger *trigger : triggers) {
		delete trigger;
	}
	mongo_connection_manager_->delete_client(con_local_);
	mongo_connection_manager_->delete_client(con_replica_);
	delete mutex_;
#ifdef USE_TIMETRACKER
	delete tt_;
#endif
}

void
EventTriggerManager::check_events()
{
	//lock to be thread safe (e.g. registration during checking)
	MutexLocker lock(mutex_);

	TIMETRACK_START(ttc_trigger_loop_);
	for (EventTrigger *trigger : triggers) {
		bool ok = true;
		try {
			auto next = trigger->change_stream.begin();
			TIMETRACK_START(ttc_callback_loop_);
			while (next != trigger->change_stream.end()) {
				//logger_->log_warn(name.c_str(), "Triggering: %s", bsoncxx::to_json(*next).c_str());
				//actually call the callback function
				TIMETRACK_START(ttc_callback_);
				trigger->callback(*next);
				next++;
				TIMETRACK_END(ttc_callback_);
			}
			TIMETRACK_END(ttc_callback_loop_);
		} catch (operation_exception &e) {
			logger_->log_error(name.c_str(), "Error while reading the change stream");
			ok = false;
		}
		// TODO Do we still need to check whether the cursor is dead?
		// (with old driver: (!ok || trigger->oplog_cursor->isDead()))
		if (!ok) {
			TIMETRACK_START(ttc_reinit_);
			if (cfg_debug_)
				logger_->log_debug(name.c_str(), "Tailable Cursor is dead, requerying");
			//check if collection is local or replicated
			client *con;
			if (std::find(dbnames_distributed_.begin(),
			              dbnames_distributed_.end(),
			              get_db_name(trigger->ns_db))
			    != dbnames_distributed_.end()) {
				con = con_replica_;
			} else {
				con = con_local_;
			}
			auto db_coll_pair = split_db_collection_string(trigger->ns);
			auto collection   = con->database(db_coll_pair.first)[db_coll_pair.second];
			try {
				trigger->change_stream = create_change_stream(collection, trigger->filter_query.view());
			} catch (mongocxx::query_exception &e) {
				logger_->log_error(name.c_str(),
				                   "Failed to create change stream, broken trigger for collection %s: %s",
				                   trigger->ns.c_str(),
				                   e.what());
			}
			TIMETRACK_END(ttc_reinit_);
		}
	}
	TIMETRACK_END(ttc_trigger_loop_);
#ifdef USE_TIMETRACKER
	if (++tt_loopcount_ % 5 == 0) {
		tt_->print_to_stdout();
	}
#endif
}

/**
 * Remove a previously registered trigger
 * @param trigger Pointer to the trigger to remove
 */
void
EventTriggerManager::remove_trigger(EventTrigger *trigger)
{
	triggers.remove(trigger);
	delete trigger;
}

change_stream
EventTriggerManager::create_change_stream(mongocxx::collection &coll, bsoncxx::document::view query)
{
	// TODO Allow non-empty pipelines
	// @body We used to have a regular mongodb query as input to the oplog, but
	// now this needs to be a pipeline. Adapt the change stream creation and the
	// robot-memory API so we also accept a non-empty pipeline.
	if (!query.empty()) {
		throw fawkes::Exception("Non-empty queries are not implemented!");
	}
	mongocxx::options::change_stream opts;
	opts.full_document("updateLookup");
	opts.max_await_time(std::chrono::milliseconds(1));
	auto res = coll.watch(opts);
	// Go to end of change stream to get new updates from then on.
	auto it = res.begin();
	while (std::next(it) != res.end()) {}

	return res;
}

/** Split database name from namespace.
 * @param ns namespace, format db.collection
 * @return db part of @p ns
 */
std::string
EventTriggerManager::get_db_name(const std::string &ns)
{
	std::string::size_type dot_pos = ns.find(".");
	if (dot_pos == std::string::npos) {
		return "";
	} else {
		return ns.substr(0, dot_pos);
	}
}
