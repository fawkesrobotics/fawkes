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

#ifdef HAVE_VISUALIZATION
#  include "visualization_thread.h"
#endif

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/tf.h>
#include <aspect/blackboard.h>
#include <aspect/aspect_provider.h>
#include <navgraph/aspect/navgraph_inifin.h>

#include <interfaces/NavigatorInterface.h>
#include <interfaces/NavPathInterface.h>

#include <navgraph/navgraph.h>
#include <utils/system/fam.h>

#include <navgraph/constraints/constraint_repo.h>

namespace fawkes {
  class Time;
}

class NavGraphThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::TransformAspect,
  public fawkes::AspectProviderAspect,
  public fawkes::FamListener
{
 public:
#ifdef HAVE_VISUALIZATION
  NavGraphThread(NavGraphVisualizationThread *vt);
#endif
  NavGraphThread();
  virtual ~NavGraphThread();

  virtual void init();
  virtual void once();
  virtual void loop();
  virtual void finalize();

  virtual void fam_event(const char *filename, unsigned int mask);

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  bool generate_plan(std::string goal);
  bool generate_plan(std::string goal, float ori);
  bool generate_plan(float x, float y, float ori, const std::string &target_name = "free-target");
  bool replan(const fawkes::NavGraphNode &start,
	      const fawkes::NavGraphNode &goal);
  void optimize_plan();
  void stop_motion();
  void start_plan();
  void send_next_goal();
  bool node_reached();
  bool node_ori_reached();
  bool node_ori_reached(const fawkes::NavGraphNode &node);
  size_t shortcut_possible();
  fawkes::LockPtr<fawkes::NavGraph> load_graph(std::string filename);
  void log_graph();
  void publish_path();


 private:
  fawkes::NavGraphAspectIniFin  navgraph_aspect_inifin_;

  std::string  cfg_graph_file_;
  std::string  cfg_base_frame_;
  std::string  cfg_global_frame_;
  std::string  cfg_nav_if_id_;
  float        cfg_resend_interval_;
  float        cfg_replan_interval_;
  float        cfg_replan_factor_;
#ifdef HAVE_VISUALIZATION
  float        cfg_visual_interval_;
#endif
  bool         cfg_monitor_file_;
  float        cfg_target_time_;
  float        cfg_target_ori_time_;
  bool         cfg_log_graph_;
  bool         cfg_abort_on_error_;
  bool         cfg_enable_path_execution_;
  bool         cfg_allow_multi_graph_;

  fawkes::NavigatorInterface *nav_if_;
  fawkes::NavigatorInterface *pp_nav_if_;
  fawkes::NavPathInterface *path_if_;

  fawkes::LockPtr<fawkes::NavGraph> graph_;

  fawkes::tf::Stamped<fawkes::tf::Pose> pose_;
  bool exec_active_;
  bool target_reached_;
  bool target_ori_reached_;
  bool target_rotating_;
  float target_time_;
  fawkes::Time *target_reached_at_;
  std::string last_node_;
  fawkes::NavGraphPath path_;
  fawkes::NavGraphPath::Traversal traversal_;
  bool constrained_plan_;

  fawkes::LockPtr<fawkes::NavGraphConstraintRepo> constraint_repo_;

  unsigned int  cmd_msgid_;
  fawkes::Time *cmd_sent_at_;
  fawkes::Time *path_planned_at_;

  fawkes::Time *error_at_;
  std::string   error_reason_;

  fawkes::FileAlterationMonitor *fam_;


#ifdef HAVE_VISUALIZATION
  fawkes::Time *visualized_at_;
  NavGraphVisualizationThread *vt_;
#endif
};

#endif
