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

#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <plugins/mongodb/aspect/mongodb_conncreator.h>
#include <core/threading/mutex.h>

#include <memory>
#include <vector>

#include "event_trigger_manager.h"
#include "computables/computables_manager.h"

namespace fawkes {
  class RobotMemoryInterface;
}
namespace mongo {
	class DBClientBase;
	class DBClientCursor;
}

///typedef for shorter type description
typedef std::unique_ptr<mongo::DBClientCursor> QResCursor;

class RobotMemory
{
  /// Friend the RobotMemoryThread so that only it can access the loop and init functions
  friend class RobotMemoryThread;

  public:
    RobotMemory(fawkes::Configuration* config, fawkes::Logger* logger,
                fawkes::Clock* clock, fawkes::MongoDBConnCreator* mongo_connection_manager,
                fawkes::BlackBoard* blackboard);
    virtual ~RobotMemory();

    //robot memory functions
    QResCursor query(mongo::Query query, const std::string& collection = "");
    mongo::BSONObj aggregate(const std::vector<mongo::BSONObj>& pipeline,
                             const std::string& collection = "");
    int insert(mongo::BSONObj obj, const std::string& collection = "");
    int insert(std::vector<mongo::BSONObj> v_obj, const std::string& collection = "");
    int insert(const std::string& obj_str, const std::string& collection = "");
    int update(mongo::Query query, mongo::BSONObj update,
               const std::string& collection = "", bool upsert = false);
    int update(mongo::Query query, const std::string& update_str,
               const std::string& collection = "", bool upsert = false);
    mongo::BSONObj find_one_and_update(const mongo::BSONObj& filter, const mongo::BSONObj& update,
                                       const std::string& collection,
                                       bool upsert = false, bool return_new = true);
    int remove(mongo::Query query, const std::string& collection = "");
    mongo::BSONObj mapreduce(mongo::Query query, const std::string& collection,
                             const std::string& js_map_fun, const std::string& js_reduce_fun);
    QResCursor aggregate(mongo::BSONObj pipeline, const std::string& collection = "");
    int drop_collection(const std::string& collection);
    int clear_memory();
    int restore_collection(const std::string& collection, const std::string& directory = "@CONFDIR@/robot-memory");
    int dump_collection(const std::string& collection, const std::string& directory = "@CONFDIR@/robot-memory");
    int create_index(mongo::BSONObj keys, const std::string& collection = "", bool unique = false);

    //bool semaphore_create(const std::string& name, unsigned int value);
    //bool semaphore_acquire(const std::string& name, unsigned int v = 1);
    //bool semaphore_release(const std::string& name, unsigned int v = 1);
    bool mutex_setup_ttl(float max_age_sec);
    bool mutex_create(const std::string& name);
    bool mutex_destroy(const std::string& name);
    bool mutex_try_lock(const std::string& name, bool force = false);
    bool mutex_try_lock(const std::string& name, const std::string& identity, bool force = false);
    bool mutex_unlock(const std::string& name, const std::string& identity);
    bool mutex_renew_lock(const std::string& name, const std::string& identity);
    bool mutex_expire_locks(float max_age_sec);

    /**
     * Register a trigger to be notified when the robot memory is updated and the updated document matches the query
     * @param query Query the updated document has to match
     * @param collection db.collection to use
     * @param callback Callback function (e.g. &Class::callback)
     * @param _obj Pointer to class the callback is a function of (usaually this)
     * @return Trigger object pointer, save it to remove the trigger later
     */
    template<typename T>
    EventTrigger* register_trigger(mongo::Query query, const std::string& collection, void(T::*callback)(mongo::BSONObj), T *_obj)
    {
      check_collection_name(collection);
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
    template<typename T>
    EventTrigger* register_trigger(const std::string& query_str, const std::string& collection, void(T::*callback)(mongo::BSONObj), T *_obj)
    {
      check_collection_name(collection);
      return register_trigger(mongo::fromjson(query_str), collection, callback, _obj);
    }
    void remove_trigger(EventTrigger* trigger);

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
    template<typename T>
    Computable* register_computable(const mongo::Query& query_to_compute,
                                    const std::string& collection,
                                    std::list<mongo::BSONObj>(T::*compute_func)(const mongo::BSONObj&, const std::string&), T *obj, double caching_time = 0.0, int priority = 0)
    {
      check_collection_name(collection);
      return computables_manager_->register_computable(query_to_compute, collection, compute_func, obj, caching_time, priority);
    }
    void remove_computable(Computable* computable);

  private:
    fawkes::MongoDBConnCreator* mongo_connection_manager_;
    mongo::DBClientBase* mongodb_client_local_;
    mongo::DBClientBase* mongodb_client_distributed_;
    bool distributed_;
    fawkes::Configuration* config_;
    fawkes::Logger* logger_;
    fawkes::Clock* clock_;
    fawkes::BlackBoard* blackboard_;

    const char* name_ = "RobotMemory";
    std::string database_name_;
    std::string default_collection_;
    bool debug_;
    fawkes::Mutex mutex_;
    fawkes::RobotMemoryInterface* rm_if_;
    EventTriggerManager* trigger_manager_;
    ComputablesManager* computables_manager_;
    std::vector<std::string> distributed_dbs_;

    unsigned int cfg_startup_grace_period_;
    std::string  cfg_coord_database_;
    std::string  cfg_coord_mutex_collection_;

    void init();
    void loop();

    void log(const std::string& what, const std::string& level = "info");
    void log_deb(const std::string& what, const std::string& level = "info");
    void log(const mongo::Query& query, const std::string& what,
             const std::string& level = "info");
    void log(const mongo::BSONObj& obj, const std::string& what,
             const std::string& level = "info");
    void log_deb(const mongo::Query& query, const std::string& what,
                 const std::string& level = "info");
    void log_deb(const mongo::BSONObj& obj, const std::string& what,
                 const std::string& level = "info");

    void set_fields(mongo::BSONObj &obj, const std::string& what);
    void set_fields(mongo::Query &q, const std::string& what);
    void remove_field(mongo::Query &q, const std::string& what);

    std::string check_collection_name(const std::string& collection);
    mongo::DBClientBase* get_mongodb_client(const std::string& collection);
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_H_ */
