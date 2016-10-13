
/***************************************************************************
 *  pddl_robot_memory_thread.cpp - pddl_robot_memory
 *
 *  Plugin created: Thu Oct 13 13:34:05 2016

 *  Copyright  2016  Frederik Zwilling
 *
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

#include "pddl_robot_memory_thread.h"
#include <fstream>
#include <utils/misc/string_conversions.h>

using namespace fawkes;

/** @class PddlRobotMemoryThread 'pddl_robot_memory_thread.h' 
 * Generate PDDL files from the robot memory
 * @author Frederik Zwilling
 */

PddlRobotMemoryThread::PddlRobotMemoryThread()
 : Thread("PddlRobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
             BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL) 
{
}

void
PddlRobotMemoryThread::init()
{
  //read config values
  collection = config->get_string("plugins/pddl-robot-memory/collection");
  input_path = StringConversions::resolve_path("@BASEDIR@/src/agents/" +
    config->get_string("plugins/pddl-robot-memory/input-problem-description"));
  output_path = StringConversions::resolve_path("@BASEDIR@/src/agents/" +
    config->get_string("plugins/pddl-robot-memory/output-problem-description"));

  //read input template of problem description
  std::string input;
  std::ifstream istream(input_path);
  if(istream.is_open())
  {
    input = std::string((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());
    istream.close();
  }
  else
  {
    logger->log_error(name(), "Could not open %s", input_path.c_str());
  }

  //TODO: expand template
  std::string output = input;

  //generate output
  logger->log_info(name(), "Output:\n%s", output.c_str());
  logger->log_info(name(), "opening: %s", input_path.c_str());
  std::ofstream ostream(output_path);
  if(ostream.is_open())
  {
    ostream << output.c_str();
    ostream.close();
  }
  else
  {
    logger->log_error(name(), "Could not open %s", output_path.c_str());
  }

  logger->log_info(name(), "Generation of PDDL problem description finished");
}

void
PddlRobotMemoryThread::loop()
{
}

void
PddlRobotMemoryThread::finalize()
{
}

