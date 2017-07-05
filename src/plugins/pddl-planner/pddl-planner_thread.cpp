
/***************************************************************************
 *  pddl-planner_thread.cpp - pddl-planner
 *
 *  Created: Wed Dec  7 19:09:44 2016
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

#include "pddl-planner_thread.h"
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <utils/misc/string_conversions.h>

using namespace fawkes;
using namespace mongo;

/** @class PddlPlannerThread 'pddl-planner_thread.h' 
 * Starts a pddl planner and writes the resulting plan into the robot memory
 * @author Frederik Zwilling
 */

/** Constructor. */
PddlPlannerThread::PddlPlannerThread()
 : Thread("PddlPlannerThread", Thread::OPMODE_WAITFORWAKEUP),
   BlackBoardInterfaceListener("PddlPlannerThread")
{
}

void
PddlPlannerThread::init()
{
  //setup interface
  gen_if = blackboard->open_for_writing<PddlPlannerInterface>(config->get_string("plugins/pddl-planner/interface-name").c_str());
  gen_if->set_msg_id(0);
  gen_if->set_final(false);
  gen_if->write();

  //setup interface listener
  bbil_add_message_interface(gen_if);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

  result_path = StringConversions::resolve_path(config->get_string("plugins/pddl-planner/description-folder") + config->get_string("plugins/pddl-planner/result-file"));
  collection = config->get_string("plugins/pddl-planner/collection");
}

/**
 * Thread is only waked up if there was a new interface message to plan
 */
void
PddlPlannerThread::loop()
{
  logger->log_info(name(), "Starting PDDL Planning...");

  //construct planner command
  std::string command = "ff -p " + config->get_string("plugins/pddl-planner/description-folder")
          + " -o " + config->get_string("plugins/pddl-planner/domain-description")
          + " -f " + config->get_string("plugins/pddl-planner/problem-description")
          + " > " + result_path;
  logger->log_info(name(), "Calling %s", command.c_str());
  FILE *bash_output = popen(command.c_str(), "r");
  //check if output is ok
  if(!bash_output)
  {
    logger->log_info(name(), "Could not plan");
    return;
  }
  logger->log_info(name(), "Finished planning");

  sleep(2);

  //Parse Result and write it into the robot memory
  logger->log_info(name(), "Parsing result");
  std::string result;
  std::ifstream istream(result_path);
  if(istream.is_open())
  {
    result = std::string((std::istreambuf_iterator<char>(istream)), std::istreambuf_iterator<char>());
    istream.close();
  }
  else
  {
    logger->log_error(name(), "Could not open %s", result_path.c_str());
    robot_memory->update(fromjson("{plan:{$exists:true}}"), fromjson("{plan:1,fail:1,steps:[]}"), collection, true);
    return;
  }
  size_t cur_pos = 0;
  if(result.find("found legal plan as follows", cur_pos) == std::string::npos)
  {
    logger->log_error(name(), "Planning Failed: %s", result.c_str());
    robot_memory->update(fromjson("{plan:{$exists:true}}"), fromjson("{plan:1,fail:1,steps:[]}"), collection, true);
    return;
  }
  //remove stuff that could confuse us later
  result.erase(result.find("time spent:", cur_pos));

  cur_pos = result.find("step", cur_pos) + 4;
  BSONObjBuilder plan_builder;
  plan_builder << "plan" << 1;
  BSONArrayBuilder steps_arr_builder;
  while(result.find(": ", cur_pos) != std::string::npos)
  {
    cur_pos = result.find(": ", cur_pos) + 2;
    size_t line_end =  result.find("\n", cur_pos);
    logger->log_info(name(), "line:%s (%zu-%zu)",
                     result.substr(cur_pos, line_end-cur_pos).c_str(), cur_pos, line_end);
    BSONObjBuilder step_builder;
    if(line_end < result.find(" ", cur_pos))
    {
      step_builder << "name" << result.substr(cur_pos, line_end - cur_pos);
    }
    else
    {
      size_t action_end =  result.find(" ", cur_pos);
      step_builder << "name" << StringConversions::to_lower(result.substr(cur_pos, action_end - cur_pos));
      cur_pos = action_end + 1;
      BSONArrayBuilder args_builder;
      while(cur_pos < line_end)
      {
        size_t arg_end =  result.find(" ", cur_pos);
        if(arg_end > line_end)
          arg_end = line_end;
        args_builder << result.substr(cur_pos, arg_end - cur_pos);
        cur_pos = arg_end + 1;
      }
      step_builder << "args" << args_builder.arr();
    }
    steps_arr_builder << step_builder.obj();
  }
  plan_builder << "steps" << steps_arr_builder.arr();
  BSONObj plan = plan_builder.obj();
  logger->log_info(name(), "Plan: %s", plan.toString().c_str());

  //Write result into Robot Memory
  robot_memory->update(fromjson("{plan:{$exists:true}}"), plan, collection, true);
}

void
PddlPlannerThread::finalize()
{
}

bool
PddlPlannerThread::bb_interface_message_received(Interface *interface, fawkes::Message *message) throw()
{
  if (message->is_of_type<PddlPlannerInterface::PlanMessage>()) {
    PddlPlannerInterface::PlanMessage* msg = (PddlPlannerInterface::PlanMessage*) message;
    gen_if->set_msg_id(msg->id());
    gen_if->set_final(false);
    gen_if->write();
    wakeup(); //activates loop where the generation is done
  } else {
    logger->log_error(name(), "Received unknown message of type %s, ignoring",
        message->type());
  }
  return false;
}
