/***************************************************************************
 *  event_trigger_manager.h - Manager to realize triggers on events in the robot memory
 *    
 *
 *  Created: 3:53:45 PM 2016
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

#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_EVENT_TRIGGER_MANAGER_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_EVENT_TRIGGER_MANAGER_H_

#include "event_trigger.h"

#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/exception.h>
#include <core/threading/mutex_locker.h>
#include <plugin/loader.h>
#include <plugins/mongodb/aspect/mongodb_conncreator.h>
#include <plugins/mongodb/utils.h>

#include <boost/bind.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <list>

class EventTriggerManager
{
	/// Access for robot memory to use the check_events function in the loop
	friend class RobotMemory;

public:
	EventTriggerManager(fawkes::Logger *            logger,
	                    fawkes::Configuration *     config,
	                    fawkes::MongoDBConnCreator *mongo_connection_manager);
	virtual ~EventTriggerManager();

	/**
     * Register a trigger to be notified when the robot memory is updated and the updated document matches the query
     * @param query Query the updated document has to match
     * @param dbcollection db.collection to use
     * @param callback Callback function (e.g. &Class::callback)
     * @param obj Pointer to class the callback is a function of (usaually this)
     * @return Trigger object pointer, save it to remove the trigger later
     */
	template <typename T>
	EventTrigger *
	register_trigger(const bsoncxx::document::view &query,
	                 std::string                    dbcollection,
	                 void (T::*callback)(const bsoncxx::document::view &),
	                 T *obj)
	{
		//logger_->log_warn("RM|EventTriggerManager", "Registering trigger on %s", dbcollection.c_str());
		//lock to be thread safe (e.g. registration during checking)
		fawkes::MutexLocker lock(mutex_);

		if (!query.empty()) {
			throw fawkes::Exception("Non-empty queries are not implemented!");
		}

		//check if dbcollection is local or replicated
		mongocxx::client *con;
		if (std::find(dbnames_distributed_.begin(),
		              dbnames_distributed_.end(),
		              get_db_name(dbcollection))
		    != dbnames_distributed_.end()) {
			con = con_replica_;
		} else {
			con = con_local_;
		}
		auto          db_coll_pair = split_db_collection_string(dbcollection);
		auto          collection   = con->database(db_coll_pair.first)[db_coll_pair.second];
		EventTrigger *trigger      = new EventTrigger(create_change_stream(collection, query),
                                             query,
                                             dbcollection,
                                             boost::bind(callback, obj, _1));
		triggers.push_back(trigger);
		return trigger;
	}

	void remove_trigger(EventTrigger *trigger);

	static std::string get_db_name(const std::string &ns);

private:
	void check_events();
	mongocxx::change_stream
	create_change_stream(mongocxx::collection &  collection,
	                     bsoncxx::document::view query,
	                     mongocxx::stdx::optional<bsoncxx::document::view_or_value> =
	                       mongocxx::stdx::optional<bsoncxx::document::view_or_value>());

	std::string            name = "RobotMemory EventTriggerManager";
	fawkes::Logger *       logger_;
	fawkes::Configuration *config_;
	fawkes::Mutex *        mutex_;

	fawkes::MongoDBConnCreator *mongo_connection_manager_;
	//MongoDB connections (necessary because the mongos instance used by the robot memory has no access to the oplog)
	mongocxx::client *con_local_;
	mongocxx::client *con_replica_;

	std::vector<std::string> dbnames_distributed_;
	std::vector<std::string> dbnames_local_;
	bool                     cfg_debug_;

	std::list<EventTrigger *> triggers;
};

#endif //FAWKES_SRC_PLUGINS_ROBOT_MEMORY_EVENT_TRIGGER_MANAGER_H_
