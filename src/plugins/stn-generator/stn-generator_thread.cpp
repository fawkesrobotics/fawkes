
/***************************************************************************
 *  stn-generator_thread.cpp - stn-generator
 *
 *  Created: Sat May  6 20:16:21 2017
 *  Copyright  2017  Matthias Loebach
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

#include <fstream>
#include <streambuf>
#include <utils/misc/string_conversions.h>
#include <chrono>
#include <thread>

#include "stn-generator_thread.h"

using namespace fawkes;
using namespace mongo;

/** @class StnGeneratorThread 'stn-generator_thread.h' 
 * Generates an STN representation of a sequential task plan
 * @author Matthias Loebach
 */

/** Constructor. */
StnGeneratorThread::StnGeneratorThread()
 : Thread("StnGeneratorThread", Thread::OPMODE_WAITFORWAKEUP),
  BlackBoardInterfaceListener("StnGeneratorThread")
{
}

void
StnGeneratorThread::init()
{
  logger->log_info(name(),"reading config");
  std::string cfg_prefix = "plugins/stn-generator/";
  cfg_plan_collection_ = config->get_string(cfg_prefix + "plan/collection");
  cfg_output_collection_ = config->get_string(cfg_prefix + "output/collection");
  cfg_publish_to_robot_memory_ = config->get_bool(cfg_prefix + "output/publish-to-rm");
  cfg_draw_graph_ = config->get_bool(cfg_prefix + "output/draw-graph");

  std::string pddl_domain_path = StringConversions::resolve_path(
      config->get_string(cfg_prefix + "domain-file"));
  cfg_pddl_problem_path_ = StringConversions::resolve_path(
      config->get_string(cfg_prefix + "problem-file"));

  std::ifstream s(pddl_domain_path);
  if ( ! s.good() ) {
    logger->log_error(name(), "Could not open domain-file at %s", pddl_domain_path.c_str());
  }
  std::string pddl_domain;

  s.seekg(0, std::ios::end);
  pddl_domain.reserve(s.tellg());
  s.seekg(0, std::ios::beg);
  pddl_domain.assign((std::istreambuf_iterator<char>(s)),
      std::istreambuf_iterator<char>());

  if ( config->get_bool(cfg_prefix + "generate-classic-domain") ) {
    std::string classic_dom_path = StringConversions::resolve_path(
      config->get_string(cfg_prefix + "classic-domain-file"));
    stn_ = new stn::Stn(logger, classic_dom_path);
  } else {
    stn_ = new stn::Stn(logger);
  }
  stn_->set_pddl_domain(pddl_domain);
  logger->log_info(name(),"Created STN object from domain");

  plan_if_ = blackboard->open_for_reading<PddlPlannerInterface>(config->get_string(cfg_prefix + "plan/interface").c_str());
  bbil_add_data_interface(plan_if_);
  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
}

void
StnGeneratorThread::loop()
{
  std::ifstream s(cfg_pddl_problem_path_);
  if ( ! s.good() ) {
    logger->log_error(name(), "Could not open problem-file at %s", cfg_pddl_problem_path_.c_str());
  }
  std::string pddl_problem;
  s.seekg(0, std::ios::end);
  pddl_problem.reserve(s.tellg());
  s.seekg(0, std::ios::beg);
  pddl_problem.assign((std::istreambuf_iterator<char>(s)),
      std::istreambuf_iterator<char>());
  stn_->read_initial_state(pddl_problem);

  QResCursor cursor = robot_memory->query(fromjson("{plan:1}"), cfg_plan_collection_);
  while ( cursor->more() ) {
    BSONObj obj = cursor->next();
    std::vector<BSONElement> actions = obj.getField("actions").Array();
    for ( auto &a : actions ) {
      BSONObj o = a.Obj();
      std::string args;
      bool first = true;
      for ( auto &arg : o.getField("args").Array() ) {
        if ( !first ) {
          args += " ";
        }
        first = false;
        args += arg.str();
      }
      stn_->add_plan_action(o.getField("name").str(), args);
      logger->log_debug(name(), "Added Plan action %s to STN", o.getField("name").str().c_str());
    }
  }
  stn_->generate();
  if ( cfg_draw_graph_ ) {
    //FIXME this throws an out_of_range exception in larger graphs
    stn_->drawGraph();
  }
  logger->log_info(name(), "STN Generation finished.");

  if ( cfg_publish_to_robot_memory_ ) {
    //TODO reset actions in robot-memory
    for ( auto& action : stn_->get_bson() ) {
      BSONObjBuilder rm_action;
      rm_action << "relation" << "proposed-stn-action";
      rm_action.appendElements(action);
      robot_memory->insert(rm_action.obj(), cfg_output_collection_);
    }
    // ensure all actions are written to RM before acknowledment
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    num_published_actions_ += stn_->get_bson().size();
    BSONObjBuilder rm_final;
    rm_final << "relation" << "stn-sync";
    rm_final << "state" << "synced";
    rm_final << "count" << std::to_string(num_published_actions_);
    robot_memory->insert(rm_final.obj(), cfg_output_collection_);
  }
}

void
StnGeneratorThread::finalize()
{
  delete stn_;
}

void
StnGeneratorThread::bb_interface_data_changed(Interface *interface) throw()
{
  if ( interface->uid() == plan_if_->uid() ) {
    plan_if_->read();
    if ( plan_if_->is_final() ) {
      logger->log_info(name(), "Planning is final, starting STN generation");
      wakeup();
    }
  } else {
    logger->log_error(name(), "Received data change for wrong interface");
  }
}
