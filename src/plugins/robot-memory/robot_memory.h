/***************************************************************************
 *  robot_memory.h - Class for storing and querying information in the RobotMemory
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
#ifndef _PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_H_
#define _PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_H_

#include "computables/computables_manager.h"
#include "event_trigger_manager.h"

#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <core/threading/mutex.h>
#include <plugins/mongodb/aspect/mongodb_conncreator.h>

#include <bsoncxx/json.hpp>
#include <memory>
#include <utility>
#include <vector>

namespace fawkes {
class RobotMemoryInterface;
#ifdef USE_TIMETRACKER
class TimeTracker;
#endif
} // namespace fawkes

class RobotMemory
{
	/// Friend the RobotMemoryThread so that only it can access the loop and init functions
	friend class RobotMemoryThread;

public:
	RobotMemory(fawkes::Configuration *     config,
	            fawkes::Logger *            logger,
	            fawkes::Clock *             clock,
	            fawkes::MongoDBConnCreator *mongo_connection_manager,
	            fawkes::BlackBoard *        blackboard);
	virtual ~RobotMemory();

	//robot memory functions
	mongocxx::cursor query(bsoncxx::document::view query,
	                       const std::string &     collection_name = "",
	                       mongocxx::options::find query_options   = mongocxx::options::find());
	// TODO fix int return codes, should be booleans
	int insert(bsoncxx::document::view, const std::string &collection = "");
	int insert(std::vector<bsoncxx::document::view> v_obj, const std::string &collection = "");
	int insert(const std::string &obj_str, const std::string &collection = "");
	int update(const bsoncxx::document::view &query,
	           const bsoncxx::document::view &update,
	           const std::string &            collection = "",
	           bool                           upsert     = false);
	int update(const bsoncxx::document::view &query,
	           const std::string &            update_str,
	           const std::string &            collection = "",
	           bool                           upsert     = false);
	bsoncxx::document::value find_one_and_update(const bsoncxx::document::view &filter,
	                                             const bsoncxx::document::view &update,
	                                             const std::string &            collection,
	                                             bool                           upsert     = false,
	                                             bool                           return_new = true);
	int remove(const bsoncxx::document::view &query, const std::string &collection = "");
	bsoncxx::document::value mapreduce(const bsoncxx::document::view &query,
	                                   const std::string &            collection,
	                                   const std::string &            js_map_fun,
	                                   const std::string &            js_reduce_fun);
	mongocxx::cursor aggregate(mongocxx::pipeline &pipeline, const std::string &collection_name = "");
	int              drop_collection(const std::string &collection);
	int              clear_memory();
	int              restore_collection(const std::string &dbcollection,
	                                    const std::string &directory = "@CONFDIR@/robot-memory",
	                                    std::string        target_dbcollection = "");
	int              dump_collection(const std::string &dbcollection,
	                                 const std::string &directory = "@CONFDIR@/robot-memory");
	int              create_index(bsoncxx::document::view keys,
	                              const std::string &     collection = "",
	                              bool                    unique     = false);

	//bool semaphore_create(const std::string& name, unsigned int value);
	//bool semaphore_acquire(const std::string& name, unsigned int v = 1);
	//bool semaphore_release(const std::string& name, unsigned int v = 1);
	bool mutex_setup_ttl(float max_age_sec);
	bool mutex_create(const std::string &name);
	bool mutex_destroy(const std::string &name);
	bool mutex_try_lock(const std::string &name, bool force = false);
	bool mutex_try_lock(const std::string &name, const std::string &identity, bool force = false);
	bool mutex_unlock(const std::string &name, const std::string &identity);
	bool mutex_renew_lock(const std::string &name, const std::string &identity);
	bool mutex_expire_locks(float max_age_sec);

	/**
     * Register a trigger to be notified when the robot memory is updated and the updated document matches the query
     * @param query Query the updated document has to match
     * @param collection db.collection to use
     * @param callback Callback function (e.g. &Class::callback)
     * @param _obj Pointer to class the callback is a function of (usaually this)
     * @return Trigger object pointer, save it to remove the trigger later
     */
	template <typename T>
	EventTrigger *
	register_trigger(const bsoncxx::document::view &query,
	                 const std::string &            collection,
	                 void (T::*callback)(const bsoncxx::document::view &),
	                 T *_obj)
	{
		return trigger_manager_->register_trigger(query, collection, callback, _obj);
	}
	/**
     * Register a trigger to be notified when the robot memory is updated and the updated document matches the query
     * @param query_str Query as JSON string
     * @param collection db.collection to use
     * @param callback Callback function (e.g. &Class::callback)
     * @param _obj Pointer to class the callback is a function of (usaually this)
     * @return Trigger object pointer, save it to remove the trigger later
     */
	template <typename T>
	EventTrigger *
	register_trigger(const std::string &query_str,
	                 const std::string &collection,
	                 void (T::*callback)(bsoncxx::document::value),
	                 T *_obj)
	{
		return register_trigger(bsoncxx::from_json(query_str), collection, callback, _obj);
	}
	void remove_trigger(EventTrigger *trigger);

	/**
     * Registers a Computable which provides information in the robot memory that is computed on demand.
     *
     * @param query_to_compute Query describing what the function computes. Yor computable is called when an new query matches the key value fields in the identifiyer.
     * @param collection db.collection to fill with computed information
     * @param compute_func Callback function that computes the information and retruns a list of computed documents
     * @param obj Pointer to class the callback is a function of (usaually this)
     * @param caching_time How long should computed results for a query be cached and be used for identical queries in that time?
     * @param priority Computable priority ordering the evaluation
     * @return Computable Object pointer used for removing it
     */
	template <typename T>
	Computable *
	register_computable(bsoncxx::document::value &&query_to_compute,
	                    const std::string &        collection,
	                    std::list<bsoncxx::document::value> (
	                      T::*compute_func)(const bsoncxx::document::view &, const std::string &),
	                    T *    obj,
	                    double caching_time = 0.0,
	                    int    priority     = 0)
	{
		return computables_manager_->register_computable(
		  std::move(query_to_compute), collection, compute_func, obj, caching_time, priority);
	}
	void remove_computable(Computable *computable);

private:
	fawkes::MongoDBConnCreator *mongo_connection_manager_;
	mongocxx::client *          mongodb_client_local_;
	mongocxx::client *          mongodb_client_distributed_;
	bool                        distributed_;
	fawkes::Configuration *     config_;
	fawkes::Logger *            logger_;
	fawkes::Clock *             clock_;
	fawkes::BlackBoard *        blackboard_;

	const char *                  name_ = "RobotMemory";
	std::string                   database_name_;
	std::string                   default_collection_;
	bool                          debug_;
	fawkes::Mutex                 mutex_;
	fawkes::RobotMemoryInterface *rm_if_;
	EventTriggerManager *         trigger_manager_;
	ComputablesManager *          computables_manager_;
	std::vector<std::string>      distributed_dbs_;

	std::string cfg_coord_database_;
	std::string cfg_coord_mutex_collection_;

	void init();
	void loop();

	// TODO make log level an enum (if we need it at all)
	void log(const std::string &what, const std::string &level = "info");
	void log_deb(const std::string &what, const std::string &level = "info");
	void log(const bsoncxx::document::view &query,
	         const std::string &            what,
	         const std::string &            level = "info");
	void log_deb(const bsoncxx::document::view &query,
	             const std::string &            what,
	             const std::string &            level = "info");

	bool                 is_distributed_database(const std::string &dbcollection);
	std::string          get_hostport(const std::string &dbcollection);
	mongocxx::client *   get_mongodb_client(const std::string &collection);
	mongocxx::collection get_collection(const std::string &dbcollection);

#ifdef USE_TIMETRACKER
	fawkes::TimeTracker *tt_;
	unsigned int         tt_loopcount_;
	unsigned int         ttc_events_;
	unsigned int         ttc_cleanup_;
#endif
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_H_ */
