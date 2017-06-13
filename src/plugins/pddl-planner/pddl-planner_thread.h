
/***************************************************************************
 *  pddl-planner_thread.h - pddl-planner
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

#ifndef __PLUGINS_PDDL_PLANNER_THREAD_H_
#define __PLUGINS_PDDL_PLANNER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <interfaces/PddlPlannerInterface.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>
#include <blackboard/interface_listener.h>

class PddlPlannerThread 
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::RobotMemoryAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener
{

 public:
  PddlPlannerThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() { Thread::run(); }

 private:
  struct action {
    std::string name;
    std::vector<std::string> args;
  };
  fawkes::PddlPlannerInterface *plan_if_;
	std::string cfg_descripton_path_;
	std::string cfg_result_path_;
	std::string cfg_domain_path_;
	std::string cfg_problem_path_;
	std::string cfg_fd_options_;
  std::string cfg_collection_;

  std::vector<action> action_list_;

  std::function<void()> planner_;

  void ff_planner();
  void fd_planner();
  void dbmp_planner();
  mongo::BSONObj BSONFromActionList();
  static size_t find_nth_space(const std::string& s, size_t nth);
  void print_action_list();
	std::string run_planner(std::string command);
  virtual bool bb_interface_message_received(fawkes::Interface *interface,
                                             fawkes::Message *message) throw();
};


#endif
