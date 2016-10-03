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

#include <mongo/client/dbclient.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>


///typedef for shorter type description
typedef std::unique_ptr<mongo::DBClientCursor> QResCursor;

/** @class EventTriggerManager  event_trigger_manager.h
 *
 * @author Frederik Zwilling
 */
class EventTriggerManager
{
  friend class RobotMemory;

  public:
    EventTriggerManager(fawkes::Logger* logger, fawkes::Configuration* config);
    virtual ~EventTriggerManager();

    void register_trigger(mongo::Query query, std::string collection);

  private:
    void check_events();
    QResCursor create_oplog_cursor(mongo::DBClientConnection* con, std::string oplog, mongo::Query query);

    std::string name = "RobotMemory EventTriggerManager";
    fawkes::Logger* logger_;
    fawkes::Configuration* config_;

    //MongoDB connections (necessary because the mongos instance used by the robot memory has no access to the oplog)
    mongo::DBClientConnection* con_local_;
    mongo::DBClientConnection* con_replica_;

    std::string repl_set, local_db;

    mongo::Query oplog_query;
    std::string oplog_collection;
    QResCursor oplog_cursor;
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_EVENT_TRIGGER_MANAGER_H_ */
