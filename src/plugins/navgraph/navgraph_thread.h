/***************************************************************************
 *  navgraph_thread.h - Graph-based global path planning
 *
 *  Created: Tue Sep 18 15:56:35 2012
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_NAVGRAPH_NAVGRAPH_THREAD_H_
#define __PLUGINS_NAVGRAPH_NAVGRAPH_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/blackboard.h>

#include <interfaces/NavigatorInterface.h>

#include <utils/graph/rcsoft_map_graph.h>

namespace fawkes {
  class AStar;
}

class NavGraphThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect
{
public:
  NavGraphThread();
  virtual ~NavGraphThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  void generate_plan(std::string goal);
  void generate_plan(float x, float y, float ori);
  void stop_motion();
  void start_plan();
  void send_next_goal();
  bool node_reached();


 private:
  std::string  cfg_graph_file_;
  std::string  cfg_base_frame_;
  std::string  cfg_global_frame_; 
  std::string  cfg_nav_if_id_; 
  float        cfg_tolerance_; 

  fawkes::NavigatorInterface *nav_if_;
  fawkes::NavigatorInterface *pp_nav_if_;

  fawkes::RCSoftMapGraph *map_graph_;
  fawkes::AStar *astar_;

  bool exec_active_;
  std::vector<fawkes::RCSoftMapNode> plan_;


};

#endif
