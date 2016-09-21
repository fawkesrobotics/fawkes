/***************************************************************************
 *  robot_memory_setup.cpp - Class to setup the robot memory with the mongodb cluster
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

#include "robot_memory_setup.h"
#include <mongo/client/dbclient.h>
#include <mongo/client/init.h>

using namespace fawkes;

RobotMemorySetup::RobotMemorySetup(Configuration *config, Logger *logger)
{
  this->config = config;
  this->logger = logger;
}

RobotMemorySetup::~RobotMemorySetup()
{
  shutdown_mongods();
}

/**
 * Start all necessary mongod and mongos instances to run
 * a local or distributed robot memory
 */
void RobotMemorySetup::setup_mongods()
{
  //start local mongod if necessary
  unsigned int local_port = config->get_uint("plugins/robot-memory/setup/local/port");
  if (!is_mongo_running(local_port))
  {
    const char *argv[] = {"mongod","--port", std::to_string(local_port).c_str(), NULL};
    std::string cmd = command_args_tostring(argv);
    //logger->log_debug("RobotMemorySetup", "Port: '%s'", std::to_string(local_port).c_str());
    logger->log_info("RobotMemorySetup", "Starting local mongod process: '%s'", cmd.c_str());
    local_mongod = new SubProcess("mongod-local", "mongod", argv, NULL, logger);
    logger->log_info("RobotMemorySetup", "Started local mongod");
    wait_until_started(local_port, cmd);
  }
}

/**
 * Shutdown all previously started mongod and mongos processes
 * a local or distributed robot memory
 */
void RobotMemorySetup::shutdown_mongods()
{
  if (local_mongod)
    delete local_mongod;

  //TODO:remember and shutdown Subprocesses
}

/**
 * Check if some mongo process is running on that port using netstat
 */
bool RobotMemorySetup::is_mongo_running(unsigned int port)
{
  std::string command = "netstat -an | grep " + std::to_string(port) + " | grep mongo";
  //logger->log_info("RobotMemorySetup", "Running command %s", command.c_str());
  FILE *bash_output = popen(command.c_str(), "r");
  std::string output = "";
  if(bash_output)
  {
    char buffer[100];
    while (!feof(bash_output) )
    {
      if (fgets(buffer, 100, bash_output) == NULL)
      {
        break;
      }
      output += buffer;
    }
    fclose (bash_output);
  }
  //logger->log_info("RobotMemorySetup", "Output is: %s\n", output.c_str());
  return output.length() > 0;
}

/**
 * Wait until mongod or mongos is reachable on the given port.
 * Aborts when the timeout is reached
 */
void RobotMemorySetup::wait_until_started(unsigned int port, std::string cmd, int timeout)
{
  logger->log_info("RobotMemorySetup", "Waiting until mongo is available on port: %u", port);
  int wait_step = 100000;
  for(int waited = 0; waited < timeout; waited += wait_step)
  {
    bool could_connect = false;
    try
    {
      std::string errmsg;
      mongo::DBClientConnection test_con(false);
      could_connect = test_con.connect(std::string("localhost:" + std::to_string(port)), errmsg);
      test_con.reset();
    }
    catch(Exception &e){}

    if(could_connect)
      return;
    usleep(wait_step);
  }
  logger->log_error("RobotMemorySetup", "MongoDB did not starup in the given time. Please start '%s' manually once.", cmd.c_str());
}
