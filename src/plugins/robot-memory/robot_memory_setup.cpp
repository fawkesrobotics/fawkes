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
#include <plugin/loader.h>
#include <mongo/client/dbclient.h>
#include <mongo/client/init.h>
#include <utils/misc/string_conversions.h>

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
  std::string local_db_name = config->get_string("plugins/robot-memory/database");
  std::string local_port_str = std::to_string(local_port);
  const char *local_argv[] = {"mongod", "--port", local_port_str.c_str(), NULL};
  start_mongo_process("mongod-local", local_port, local_argv);

  //only start other processes when we want to run the robot memory distributed
  if(!config->get_bool("plugins/robot-memory/setup/distributed"))
    return;

  //start config server
  unsigned int config_port = config->get_uint("plugins/robot-memory/setup/config/port");
  std::string db_path = StringConversions::resolve_path(config->get_string("plugins/robot-memory/setup/config/db-path").c_str());
  prepare_mongo_db_path(db_path);
  std::string config_port_str = std::to_string(config_port);
  const char *config_argv[] = {"mongod", "--configsvr", "--port",
      config_port_str.c_str(), "--dbpath", db_path.c_str(), NULL};
  start_mongo_process("mongod-config", config_port, config_argv);
  create_database(local_port, local_db_name);

  //start own part of replica set
  unsigned int distributed_port = config->get_uint("plugins/robot-memory/setup/replicated/port");
  std::string distributed_db_path = StringConversions::resolve_path(config->get_string("plugins/robot-memory/setup/replicated/db-path").c_str());
  prepare_mongo_db_path(distributed_db_path);
  std::string distributed_port_str = std::to_string(distributed_port);
  std::string distributed_replset = config->get_string("plugins/robot-memory/setup/replicated/replica-set-name");
  const char *distributed_argv[] = {"mongod", "--port", distributed_port_str.c_str(),
      "--dbpath", distributed_db_path.c_str(),
      "--replSet", distributed_replset.c_str(),NULL};
  start_mongo_process("mongod-replicated", distributed_port, distributed_argv);

  //configure replica set
  std::string repl_config = "{_id:'" + distributed_replset + "', members:"
      + config->get_string("plugins/robot-memory/setup/replicated/replica-set-members") + "}";
  run_mongo_command(distributed_port, std::string("{replSetInitiate:" + repl_config + "}"), "already initialized");
  //wait for replica set initialization and election
  usleep(3000000);
  create_database(distributed_port, distributed_replset);

  //start mongos for accessing
  unsigned int mongos_port = config->get_uint("plugins/robot-memory/setup/mongos/port");
  std::string mongos_port_str = std::to_string(mongos_port);
  std::string confighost = "localhost:" + config_port_str;
  const char *mongos_argv[] = {"mongos", "--port", mongos_port_str.c_str(),
      "--configdb", confighost.c_str(), NULL};
  start_mongo_process("mongos", mongos_port, mongos_argv);

  //configure mongos (add parts of the sharded cluster)
  mongo::BSONObj current_shards =  run_mongo_command(mongos_port, std::string("{listShards:1}"));
  if(current_shards.getField("shards").Array().size() == 0)
  {
    run_mongo_command(mongos_port, std::string("{addShard: 'localhost:" + local_port_str + "'}"), "host already used");
    run_mongo_command(mongos_port, std::string("{addShard: '" + distributed_replset +
      "/localhost:" + distributed_port_str + "'}"), "host already used");
  }
  //define which db is in which shard
  run_mongo_command(mongos_port, std::string("{movePrimary: '" + distributed_replset + "', to: '" + distributed_replset + "'}"), "it is already the primary");
  run_mongo_command(mongos_port, std::string("{movePrimary: '" + local_db_name + "', to: 'shard0000'}"), "it is already the primary");
}

/**
 * Start a single mongo process
 */
void RobotMemorySetup::start_mongo_process(std::string proc_name, unsigned int port, const char *argv[])
{
  if (!is_mongo_running(port))
    {
      std::string cmd = command_args_tostring(argv);
      logger->log_info("RobotMemorySetup", "Starting %s process: '%s'", proc_name.c_str(), cmd.c_str());
      config_mongod = new SubProcess(proc_name.c_str(), argv[0], argv, NULL, logger);
      logger->log_info("RobotMemorySetup", "Started %s", proc_name.c_str());
      wait_until_started(port, cmd);
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
  if (config_mongod)
    delete config_mongod;
  if (distribuded_mongod)
    delete distribuded_mongod;
  if (mongos)
    delete mongos;
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
  std::string err_msg = "MongoDB did not starup in the given time. Please start '"
      +cmd+"' manually once.";
  throw PluginLoadException("robot-memory", err_msg.c_str());
}

void RobotMemorySetup::prepare_mongo_db_path(std::string path)
{
  std::string command = "mkdir -p " + path;
  popen(command.c_str(), "r");
}

mongo::BSONObj RobotMemorySetup::run_mongo_command(unsigned int port, std::string command,
  std::string err_msg_to_ignore)
{
  std::string errmsg;
  mongo::DBClientConnection con(false);
  bool could_connect = con.connect(std::string("localhost:" + std::to_string(port)), errmsg);
  if(!could_connect)
  {
    std::string err_msg = "Could not connect to mongo process to execute command: "+ errmsg;
    throw PluginLoadException("robot-memory", err_msg.c_str());
  }
  mongo::BSONObj res;
  logger->log_info("RobotMemorySetup", "Executing db command: %s", command.c_str());
  con.runCommand("admin", mongo::fromjson(command), res);
  con.reset();
  if(res.getField("ok").type() != mongo::BSONType::NumberDouble || (res.getField("ok").Double() == 0.0 && res.getField("errmsg").String().compare(err_msg_to_ignore) != 0))
    throw PluginLoadException("robot-memory", std::string("Running DB command " + command + " failed: " + res.toString()).c_str());
  return res;
}

void RobotMemorySetup::create_database(unsigned int port, std::string name)
{
  //to create a database you have to insert at least one document
  std::string errmsg;
  mongo::DBClientConnection con(false);
  bool could_connect = con.connect(std::string("localhost:" + std::to_string(port)), errmsg);
  if(!could_connect)
  {
    std::string err_msg = "Could not connect to mongo process to execute command: "+ errmsg;
    throw PluginLoadException("robot-memory", err_msg.c_str());
  }
  mongo::BSONObj first_doc = mongo::fromjson("{initialized:1}");
  con.insert(name + ".config", first_doc);
  con.reset();


}
