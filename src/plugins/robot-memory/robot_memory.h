/***************************************************************************
 *  robot_memory.h - Class for storing and querying information in the RobotMemory
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
#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_H_

#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <memory>
#include <vector>

#include <mongo/client/dbclient.h>
#include "interfaces/RobotMemoryInterface.h"
#include "event_trigger_manager.h"
#include "computables/computables_manager.h"

namespace fawkes {
  class Mutex;
  class RobotMemoryInterface;
}

///typedef for shorter type description
typedef std::unique_ptr<mongo::DBClientCursor> QResCursor;

class RobotMemory
{
  friend class RobotMemoryThread;

  public:
    RobotMemory(fawkes::Configuration* config, fawkes::Logger* logger,
                fawkes::Clock* clock, mongo::DBClientBase* mongodb_client,
                fawkes::BlackBoard* blackboard);
    virtual ~RobotMemory();

    //robot memory functions
    QResCursor query(mongo::Query query, std::string collection = "");
    int insert(mongo::BSONObj obj, std::string collection = "");
    int insert(std::vector<mongo::BSONObj> v_obj, std::string collection = "");
    int insert(std::string obj_str, std::string collection = "");
    int update(mongo::Query query, mongo::BSONObj update, std::string collection = "", bool upsert = false);
    int update(mongo::Query query, std::string update_str, std::string collection = "", bool upsert = false);
    int remove(mongo::Query query, std::string collection = "");
    int drop_collection(std::string collection);
    int clear_memory();
    int restore_collection(std::string collection, std::string directory = "@CONFDIR@/robot-memory");
    int dump_collection(std::string collection, std::string directory = "@CONFDIR@/robot-memory");

    /**
     * Register a trigger to be notified when the robot memory is updated and the updated document matches the query
     * @param query Query the updated document has to match
     * @param collection db.collection to use
     * @param callback Callback function (e.g. &Class::callback)
     * @param _obj Pointer to class the callback is a function of (usaually this)
     * @return Trigger object pointer, save it to remove the trigger later
     */
    template<typename T>
    EventTrigger* register_trigger(mongo::Query query, std::string collection, void(T::*callback)(mongo::BSONObj), T *_obj)
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
    EventTrigger* register_trigger(std::string query_str, std::string collection, void(T::*callback)(mongo::BSONObj), T *_obj)
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
     * @return Computable Object pointer used for removing it
     */
    template<typename T>
    Computable* register_computable(mongo::Query query_to_compute, std::string collection, std::list<mongo::BSONObj>(T::*compute_func)(mongo::BSONObj, std::string), T *obj, double caching_time = 0.0, int priority = 0)
    {
      check_collection_name(collection);
      return computables_manager_->register_computable(query_to_compute, collection, compute_func, obj, caching_time, priority);
    }
    void remove_computable(Computable* computable);

  private:
    mongo::DBClientBase* mongodb_client_;
    fawkes::Configuration* config_;
    fawkes::Logger* logger_;
    fawkes::Clock* clock_;
    fawkes::BlackBoard* blackboard_;

    const char* name_ = "RobotMemory";
    std::string database_name_;
    std::string default_collection_;
    bool debug_;
    fawkes::Mutex *mutex_;
    fawkes::RobotMemoryInterface* rm_if_;
    EventTriggerManager* trigger_manager_;
    ComputablesManager* computables_manager_;

    void init();
    void loop();

    void log(std::string what, std::string level = "info");
    void log_deb(std::string what, std::string level = "info");
    void log(mongo::Query query, std::string what, std::string level = "info");
    void log(mongo::BSONObj obj, std::string what, std::string level = "info");
    void log_deb(mongo::Query query, std::string what, std::string level = "info");
    void log_deb(mongo::BSONObj obj, std::string what, std::string level = "info");

    void set_fields(mongo::BSONObj &obj, std::string what);
    void set_fields(mongo::Query &q, std::string what);
    void remove_field(mongo::Query &q, std::string what);

    void check_collection_name(std::string &collection);
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_H_ */
