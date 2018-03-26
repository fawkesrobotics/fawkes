
/***************************************************************************
 *  pddl-planner_thread.cpp - pddl-planner
 *
 *  Created: Wed Dec  7 19:09:44 2016
 *  Copyright  2016  Frederik Zwilling
 *             2017  Matthias Loebach
 *             2017  Till Hofmann
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
#include <sstream>
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
  //read config
  std::string cfg_prefix = "plugins/pddl-planner/";
	cfg_descripton_path_ = StringConversions::resolve_path(config->get_string((cfg_prefix + "description-folder")));
	cfg_result_path_ = cfg_descripton_path_ +
    config->get_string((cfg_prefix + "result-file"));
  cfg_domain_path_ = cfg_descripton_path_ + config->get_string(cfg_prefix + "domain-description");
	cfg_problem_path_ = cfg_descripton_path_ + config->get_string(cfg_prefix + "problem-description");
	cfg_fd_options_ = config->get_string(cfg_prefix + "fd-search-opts");
  cfg_collection_ = config->get_string(cfg_prefix + "collection");

  //set configured planner
  std::string planner_string = config->get_string((cfg_prefix + "planner").c_str());
	if ( planner_string == "ff" ) {
		planner_ = std::bind(&PddlPlannerThread::ff_planner, this);
		logger->log_info(name(), "Fast-Forward planner selected.");
	} else if ( planner_string == "fd" ) {
		planner_ = std::bind(&PddlPlannerThread::fd_planner, this);
		logger->log_info(name(), "Fast-Downward planner selected.");
	} else if ( planner_string == "dbmp" ) {
		planner_ = std::bind(&PddlPlannerThread::dbmp_planner, this);
		logger->log_info(name(), "DBMP selected.");
	} else {
		planner_ = std::bind(&PddlPlannerThread::ff_planner, this);
		logger->log_warn(name(), "No planner configured.\nDefaulting to ff.");
	}

  //setup interface
  plan_if_ = blackboard->open_for_writing<PddlPlannerInterface>(config->get_string(cfg_prefix + "interface-name").c_str());
  plan_if_->set_active_planner(planner_string.c_str());
  plan_if_->set_msg_id(0);
  plan_if_->set_final(false);
  plan_if_->set_success(false);
  plan_if_->write();

  //setup interface listener
  bbil_add_message_interface(plan_if_);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_MESSAGES);

  // If we receive multiple wakeup() calls during loop, only call the loop once afterwards
  // We want to only re-plan once after a loop run since multiple runs would plan on the same problem
  this->set_coalesce_wakeups(true);
}

/**
 * Thread is only waked up if there was a new interface message to plan
 */
void
PddlPlannerThread::loop()
{
  logger->log_info(name(), "Starting PDDL Planning...");

  //writes plan into action_list_
  planner_();

  if ( !action_list_.empty() ) {
    BSONObj plan = BSONFromActionList();
    robot_memory->update(fromjson("{plan:{$exists:true}}"), plan, cfg_collection_, true);
    print_action_list();
    plan_if_->set_success(true);
  } else {
    logger->log_error(name(),"Updating plan failed, action list empty!");
    robot_memory->update(fromjson("{plan:{$exists:true}}"), fromjson("{plan:0}"), cfg_collection_, true);
    plan_if_->set_success(false);
  }

  plan_if_->set_final(true);
  plan_if_->write();
}

void
PddlPlannerThread::finalize()
{
  blackboard->close(plan_if_);
}

void
PddlPlannerThread::ff_planner()
{
  logger->log_info(name(), "Starting PDDL Planning with Fast-Forward...");

  std::string command = "ff -o " + cfg_domain_path_ + " -f " + cfg_problem_path_;
  logger->log_info(name(), "Calling %s", command.c_str());
  std::string result = run_planner(command);

  //Parse Result and write it into the robot memory
  logger->log_info(name(), "Parsing result");

  action_list_.clear();

  size_t cur_pos = 0;
  if(result.find("found legal plan as follows", cur_pos) == std::string::npos) {
    logger->log_error(name(), "Planning Failed: %s", result.c_str());
    robot_memory->update(fromjson("{plan:{$exists:true}}"), fromjson("{plan:1,fail:1,steps:[]}"), cfg_collection_, true);
    return;
  }
  //remove stuff that could confuse us later
  result.erase(result.find("time spent:", cur_pos));

  cur_pos = result.find("step", cur_pos) + 4;
  while(result.find(": ", cur_pos) != std::string::npos) {
    cur_pos = result.find(": ", cur_pos) + 2;
    size_t line_end =  result.find("\n", cur_pos);
    logger->log_info(name(), "line:%s (%zu-%zu)",
                     result.substr(cur_pos, line_end-cur_pos).c_str(), cur_pos, line_end);
    action a;
    if(line_end < result.find(" ", cur_pos)) {
       a.name = result.substr(cur_pos, line_end - cur_pos);
    } else {
      size_t action_end =  result.find(" ", cur_pos);
      a.name = StringConversions::to_lower(result.substr(cur_pos, action_end - cur_pos));
      cur_pos = action_end + 1;
      while(cur_pos < line_end) {
        size_t arg_end =  result.find(" ", cur_pos);
        if(arg_end > line_end) {
          arg_end = line_end;
        }
        a.args.push_back(result.substr(cur_pos, arg_end - cur_pos));
        cur_pos = arg_end + 1;
      }
    }
    action_list_.push_back(a);
  }
}

void
PddlPlannerThread::dbmp_planner()
{
  logger->log_info(name(), "Starting PDDL Planning with DBMP...");

  std::string command = "dbmp.py -p ff --output plan.pddl " + cfg_domain_path_ +
    " " + cfg_problem_path_;
  logger->log_info(name(), "Calling %s", command.c_str());
  std::string result = run_planner(command);

  //Parse Result and write it into the robot memory
  logger->log_info(name(), "Parsing result");

  size_t cur_pos = 0;
  if(result.find("Planner failed", cur_pos) != std::string::npos) {
    logger->log_error(name(), "Planning Failed: %s", result.c_str());
    robot_memory->update(fromjson("{plan:{$exists:true}}"),
        fromjson("{plan:1,fail:1,steps:[]}"), cfg_collection_, true);
    return;
  }
  std::ifstream planfile("plan.pddl");
  std::string line;
  action_list_.clear();
  while (std::getline(planfile, line)) {
    std::string time_string = "Time";
    if (line.compare(0, time_string.size(), time_string) == 0) {
      // makespan, skip
      continue;
    }
    if (line[0] != '(' || line[line.size() - 1] != ')') {
      logger->log_error(name(), "Expected parantheses in line '%s'!",
          line.c_str());
      return;
    }
    // remove parantheses
    std::string action_str = line.substr(1, line.size() - 2);
    action a;
    cur_pos = action_str.find(" ", cur_pos + 1);
    a.name = StringConversions::to_lower(action_str.substr(0, cur_pos));
    size_t word_start;
    while (cur_pos != std::string::npos) {
      word_start = cur_pos + 1;
      cur_pos = action_str.find(" ", word_start);
      a.args.push_back(action_str.substr(word_start, cur_pos - word_start));
    }
    action_list_.push_back(a);
  }
}

void
PddlPlannerThread::fd_planner()
{
  logger->log_info(name(), "Starting PDDL Planning with Fast-Downward...");

	std::string command = "fast-downward"
		+ std::string(" ") + cfg_domain_path_
		+ std::string(" ") + cfg_problem_path_;

	if ( !cfg_fd_options_.empty() ) {
		command += std::string(" ") + cfg_fd_options_;
	}

	std::string result = run_planner(command);

  logger->log_info(name(),"Removing temporary planner output.");
  std::remove("output");
  std::remove("output.sas");

  size_t cur_pos = 0;
  if ( result.find("Solution found!", cur_pos) == std::string::npos) {
    logger->log_error(name(), "Planning Failed: %s", result.c_str());
    //TODO handle with correct exception
    throw;
  } else {
    cur_pos = result.find("Solution found!", cur_pos);
    cur_pos = result.find("\n", cur_pos);
    cur_pos = result.find("\n", cur_pos+1);
    logger->log_info(name(), "Planner found solution.");
  }
  result.erase(0,cur_pos);
  size_t end_pos = result.find("Plan length: ");
  result.erase(end_pos, result.size()-1);

  std::istringstream iss(result);
  std::string line;
  // remove surplus line
  getline(iss, line);
  while ( getline(iss, line) ) {
    action a;
    a.name = line.substr(0,find_nth_space(line, 1));
    if ( find_nth_space(line, 2) != line.rfind(' ') + 1 ) {
      std::stringstream ss(line.substr(find_nth_space(line, 2),
         line.rfind(' ') - find_nth_space(line, 2)));
      std::string item;
      while (getline(ss, item, ' ')) {
        a.args.push_back(item);
      }
    }
    action_list_.push_back(a);
  }

}

BSONObj
PddlPlannerThread::BSONFromActionList()
{
  BSONObjBuilder plan_builder;
  plan_builder << "plan" << 1;
  plan_builder << "msg_id" << plan_if_->msg_id();
  BSONArrayBuilder action_arr_builder;
  for ( action  a : action_list_ ) {
    BSONObjBuilder action_builder;
    action_builder << "name" << a.name;
    BSONArrayBuilder args_builder;
    for ( std::string args : a.args ) {
      args_builder << args;
    }
    action_builder << "args" << args_builder.arr();
    action_arr_builder << action_builder.obj();
  }
  
  plan_builder << "actions" << action_arr_builder.arr();

  return plan_builder.obj();
}

size_t
PddlPlannerThread::find_nth_space(const std::string& s, size_t nth)
{
  size_t pos = 0;
  unsigned occurrence = 0;

  while (occurrence != nth && (pos = s.find(' ', pos + 1)) != std::string::npos) {
    ++occurrence;
  }

  return pos + 1;
}

void
PddlPlannerThread::print_action_list()
{
  unsigned int count = 0;
  for ( action a : action_list_ ) {
    count++;
    std::string args;
    for ( std::string arg : a.args ) {
      args += arg + " ";
    }
    logger->log_info(name(),"Action %d %s with args %s", count, a.name.c_str(), args.c_str());
  }
}

std::string
PddlPlannerThread::run_planner(std::string command)
{
	logger->log_info(name(), "Running planner with command: %s", command.c_str());
  std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) throw std::runtime_error("popen() failed!");
  char buffer[128];
  std::string result;
  while (!feof(pipe.get())) {
    if (fgets(buffer, 128, pipe.get()) != NULL)
      result += buffer;
  } 
  logger->log_info(name(), "Planner finished run.");

  return result;
}

bool
PddlPlannerThread::bb_interface_message_received(Interface *interface, fawkes::Message *message) throw()
{
  if (message->is_of_type<PddlPlannerInterface::PlanMessage>()) {
    PddlPlannerInterface::PlanMessage* msg = (PddlPlannerInterface::PlanMessage*) message;
    plan_if_->set_msg_id(msg->id());
    plan_if_->set_success(false);
    plan_if_->set_final(false);
    plan_if_->write();
    wakeup(); //activates loop where the generation is done
  } else {
    logger->log_error(name(), "Received unknown message of type %s, ignoring",
        message->type());
  }
  return false;
}
