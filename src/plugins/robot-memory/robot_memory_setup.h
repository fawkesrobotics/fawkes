/***************************************************************************
 *  robot_memory_setup.h - Class to setup the robot memory with the mongodb cluster
 *    
 *
 *  Created: 8:18:32 PM 2016
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

#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_SETUP_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_SETUP_H_

#include <config/config.h>
#include <logging/logger.h>
#include <utils/sub_process/proc.h>
#include <utils/misc/string_commands.h>
#include <mongo/client/dbclient.h>

/** @class RobotMemorySetup  robot_memory_setup.h
 *
 * @author Frederik Zwilling
 */
class RobotMemorySetup
{
  public:
    RobotMemorySetup(fawkes::Configuration *config, fawkes::Logger *logger);
    virtual ~RobotMemorySetup();

    void setup_mongods();
    void shutdown_mongods();

  private:
    fawkes::Configuration *config;
    fawkes::Logger *logger;

    bool is_mongo_running(unsigned int port);
    void wait_until_started(unsigned int port, std::string cmd, int timout = 15000000);
    void prepare_mongo_db_path(std::string path);
    void start_mongo_process(std::string proc_name, unsigned int port, const char *argv[]);
    mongo::BSONObj run_mongo_command(unsigned int port, std::string command, std::string err_msg_to_ignore="");

    fawkes::SubProcess *local_mongod;
    fawkes::SubProcess *config_mongod;
    fawkes::SubProcess *mongos;
    fawkes::SubProcess *distribuded_mongod;
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_ROBOT_MEMORY_SETUP_H_ */
