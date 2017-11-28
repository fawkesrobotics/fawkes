
/***************************************************************************
 *  stn-generator_thread.h - stn-generator
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

#ifndef __PLUGINS_STN_GENERATOR_THREAD_H_
#define __PLUGINS_STN_GENERATOR_THREAD_H_

#include <mongo/client/dbclient.h>

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <blackboard/interface_listener.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>
#include <interfaces/PddlPlannerInterface.h>

#include "stn.h"

namespace fawkes {
  // add forward declarations here, e.g., interfaces
}

class StnGeneratorThread 
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlackBoardInterfaceListener,
  public fawkes::RobotMemoryAspect
{

 public:
  StnGeneratorThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() { Thread::run(); }

 private:
   std::string cfg_plan_collection_;
   bool cfg_publish_to_robot_memory_;
   bool cfg_draw_graph_;
   size_t num_published_actions_ = 0;
   std::string cfg_output_collection_;
   std::string cfg_pddl_problem_path_;
   fawkes::PddlPlannerInterface *plan_if_;

   fawkes::stn::Stn* stn_;
};


#endif
