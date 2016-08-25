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

#include <mongo/client/dbclient.h>
#include "interfaces/RobotMemoryInterface.h"

namespace fawkes {
  class Mutex;
  class RobotMemoryInterface;
}

///typedef for shorter type description
typedef std::unique_ptr<mongo::DBClientCursor> QResCursor;

/*
 *
 */
class RobotMemory
{
  friend class RobotMemoryThread;

  public:
    RobotMemory(fawkes::Configuration* config, fawkes::Logger* logger,
                fawkes::Clock* clock, mongo::DBClientBase* mongodb_client,
                fawkes::BlackBoard* blackboard);
    virtual ~RobotMemory();

    //robot memory functions
    QResCursor query(std::string query, std::string collection = "");
    int insert(std::string insert, std::string collection = "");
    int update(std::string query, std::string update, std::string collection = "");
    int remove(std::string query, std::string collection = "");

  private:
    mongo::DBClientBase* mongodb_client_;
    fawkes::Configuration* config_;
    fawkes::Logger* logger_;
    fawkes::Clock* clock_;
    fawkes::BlackBoard* blackboard_;

    const char* name_ = "RobotMemory";
    std::string default_collection_;
    bool debug_;
    fawkes::Mutex *mutex_;
    fawkes::RobotMemoryInterface* rm_if_;

    void init();

    void log(mongo::Query query, std::string what);
    void log(mongo::BSONObj obj, std::string what);

    void set_fields(mongo::BSONObj &obj, std::string what);
    void set_fields(mongo::Query &q, std::string what);
    void remove_field(mongo::Query &q, std::string what);
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_H_ */
