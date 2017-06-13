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
#include <libs/core/exceptions/system.h>
#include <cstdio>
#include <cstdlib>

using namespace fawkes;

/** @class RobotMemorySetup  robot_memory_setup.h
 * Class to setup the robot memory with the mongodb cluster
 * @author Frederik Zwilling
 */

/**
 * Constructor of class performing the mongodb setup for the robot memory
 * @param config Configuration
 * @param logger Logger
 */
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
	std::string log_path = config->get_string("plugins/robot-memory/setup/log-path");
  prepare_mongo_db_path(log_path);

  //start local mongod if necessary
  unsigned int local_port = config->get_uint("plugins/robot-memory/setup/local/port");
  std::string local_db_name = config->get_string("plugins/robot-memory/database");
  std::string local_repl_name = config->get_string("plugins/robot-memory/setup/local/replica-set-name");
  std::string local_port_str = std::to_string(local_port);
  std::string local_db_path = StringConversions::resolve_path(config->get_string("plugins/robot-memory/setup/local/db-path").c_str());
  std::string local_log_path = StringConversions::resolve_path(log_path+"/local.log");
  std::string oplog_size = std::to_string(config->get_int("plugins/robot-memory/setup/oplog-size"));
  prepare_mongo_db_path(local_db_path);
  const char *local_argv[] =
	  {"mongod",
	   "--port", local_port_str.c_str(),
	   "--replSet", local_repl_name.c_str(),
	   "--dbpath", local_db_path.c_str(),  "--nojournal",
	   "--oplogSize", oplog_size.c_str(), //local replica set just to enable the oplog
	   "--logappend", "--logpath", local_log_path.c_str(),
	   NULL};
  local_mongod = start_mongo_process("mongod-local", local_port, local_argv);
  std::string local_config = "{_id: '" + local_repl_name + "', members:[{_id:1,host:'localhost:" + local_port_str + "'}]}";
  run_mongo_command(local_port, std::string("{replSetInitiate:" + local_config + "}"), "already initialized");
  //wait for initialization
  usleep(1000000);

  //only start other processes when we want to run the robot memory distributed
  if(!config->get_bool("plugins/robot-memory/setup/distributed-create"))
    return;

  //start own part of replica set
  unsigned int distributed_port = config->get_uint("plugins/robot-memory/setup/replicated/port");
  std::string distributed_db_path = StringConversions::resolve_path(config->get_string("plugins/robot-memory/setup/replicated/db-path").c_str());
  std::string distributed_log_path = StringConversions::resolve_path(log_path+"/distributed.log");
  prepare_mongo_db_path(distributed_db_path);
  std::string distributed_port_str = std::to_string(distributed_port);
  std::string distributed_replset = config->get_string("plugins/robot-memory/setup/replicated/replica-set-name");
  const char *distributed_argv[] =
	  {"mongod",
	   "--port", distributed_port_str.c_str(),
	   "--dbpath", distributed_db_path.c_str(),
	   "--replSet", distributed_replset.c_str(),  "--nojournal",
	   "--oplogSize", oplog_size.c_str(),
	   "--logappend", "--logpath", distributed_log_path.c_str(),
	   NULL};
  distribuded_mongod = start_mongo_process("mongod-replicated", distributed_port, distributed_argv);

  //configure replica set
	// * 1000000: sec -> microsec
  int max_setup_time =
	  config->get_int("plugins/robot-memory/setup/replicated/max_setup_time_per_host_sec") * 1000000;
  
  if (config->get_bool("plugins/robot-memory/setup/replicated/initiate"))
  {
	  std::vector<std::string> repl_set_members =
		  config->get_strings("plugins/robot-memory/setup/replicated/replica-set-members");
	  for (size_t i = 0; i < repl_set_members.size(); ++i) {
		  wait_until_started(repl_set_members[i], "repl set connect", max_setup_time, 1000000);
	  }
	  usleep(random() % 15000000);

	  std::string repl_config = "{_id:'" + distributed_replset + "', members: [";
	  for (size_t i = 0; i < repl_set_members.size(); ++i) {
		  repl_config += "{_id: " + std::to_string(i+1) + ", host: '" + repl_set_members[i] + "'}";
		  if (i < repl_set_members.size()-1) { repl_config += ", "; }
	  }
    repl_config += "]}";
	  run_mongo_command(distributed_port, std::string("{replSetInitiate:" + repl_config + "}"), "already initialized");
  }
  //wait for replica set initialization and election
  usleep(3000000);
}

/**
 * Start a single mongo process
 */
fawkes::SubProcess* RobotMemorySetup::start_mongo_process(std::string proc_name, unsigned int port, const char *argv[])
{
  if (!is_mongo_running(port))
    {
      std::string cmd = command_args_tostring(argv);
      logger->log_warn("RobotMemorySetup", "Starting %s process: '%s'", proc_name.c_str(), cmd.c_str());
      fawkes::SubProcess * process = new SubProcess(proc_name.c_str(), argv[0], argv, NULL, logger);
      logger->log_info("RobotMemorySetup", "Started %s", proc_name.c_str());
      wait_until_started(std::string("localhost:" + std::to_string(port)),
                         cmd, config->get_int("plugins/robot-memory/setup/max_setup_time"));
      return process;
    }
  return NULL;
}

/**
 * Shutdown all previously started mongod and mongos processes
 * a local or distributed robot memory
 */
void RobotMemorySetup::shutdown_mongods()
{
  if(local_mongod)
    delete local_mongod;
  if(distribuded_mongod)
    delete distribuded_mongod;
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
void RobotMemorySetup::wait_until_started(const std::string &hostport,
                                          const std::string cmd, int timeout, int wait_step)
{
	logger->log_info("RobotMemorySetup", "Waiting until mongo is available at '%s'", hostport.c_str());
  for(int waited = 0; waited < timeout; waited += wait_step)
  {
    bool could_connect = false;
    try
    {
      std::string errmsg;
      mongo::DBClientConnection test_con(false);
      could_connect = test_con.connect(hostport, errmsg);
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
  FILE *pipe = ::popen(command.c_str(), "r");
  char *line = nullptr;
  size_t n = 0;
  int len  __attribute__((unused)) = ::getline(&line, &n, pipe);
  std::string sline(line);
  free(line);
  if(::pclose(pipe))
    throw FileWriteException(path.c_str(), (std::string("Trying to `mkdir -p': ") + sline).c_str());
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
  {
    if(res.getField("errmsg").String() == "server is not running with --replSet")
      logger->log_error("RobotMemorySetup", "The mongod instance which is already running is started with the wrong parameters. Stop it to let it start by the robot memory or change the mongod config and restart it.");
    throw PluginLoadException("robot-memory", std::string("Running DB command " + command + " failed: " + res.toString()).c_str());
  }
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
