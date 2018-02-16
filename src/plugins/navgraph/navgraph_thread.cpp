/***************************************************************************
 *  navgraph_thread.cpp - Graph-based global path planning
 *
 *  Created: Tue Sep 18 16:00:34 2012
 *  Copyright  2012-2014  Tim Niemueller [www.niemueller.de]
 *                  2014  Tobias Neumann
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

#include "navgraph_thread.h"

#include <navgraph/yaml_navgraph.h>
#include <navgraph/constraints/constraint_repo.h>
#include <utils/math/angle.h>
#include <tf/utils.h>
#include <core/utils/lockptr.h>

#include <fstream>
#include <cmath>

using namespace fawkes;

/** @class NavGraphThread "navgraph_thread.h"
 * Thread to perform graph-based path planning.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphThread::NavGraphThread()
  : Thread("NavGraphThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    AspectProviderAspect(&navgraph_aspect_inifin_)
{
#ifdef HAVE_VISUALIZATION
  vt_ = NULL;
#endif
}

#ifdef HAVE_VISUALIZATION
/** Constructor. */
NavGraphThread::NavGraphThread(NavGraphVisualizationThread *vt)
  : Thread("NavGraphThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT),
    AspectProviderAspect(&navgraph_aspect_inifin_)
{
  vt_ = vt;
}
#endif

/** Destructor. */
NavGraphThread::~NavGraphThread()
{
}

void
NavGraphThread::init()
{
  try {
    cfg_graph_file_      = config->get_string("/navgraph/graph_file");
  } catch (Exception &e) {
    logger->log_warn(name(), "No graph file given, will create empty one");
  }
  cfg_base_frame_      = config->get_string("/frames/base");
  cfg_global_frame_    = config->get_string("/frames/fixed");
  cfg_nav_if_id_       = config->get_string("/navgraph/navigator_interface_id");
  cfg_resend_interval_ = config->get_float("/navgraph/resend_interval");
  cfg_replan_interval_ = config->get_float("/navgraph/replan_interval");
  cfg_replan_factor_   = config->get_float("/navgraph/replan_cost_factor");
  cfg_target_time_     = config->get_float("/navgraph/target_time");
  cfg_target_ori_time_ = config->get_float("/navgraph/target_ori_time");
  cfg_log_graph_       = config->get_bool("/navgraph/log_graph");
  cfg_abort_on_error_  = config->get_bool("/navgraph/abort_on_error");
#ifdef HAVE_VISUALIZATION
  cfg_visual_interval_ = config->get_float("/navgraph/visualization_interval");
#endif
  cfg_monitor_file_ = false;
  try {
    cfg_monitor_file_ = config->get_bool("/navgraph/monitor_file");
  } catch (Exception &e) {} // ignored

  cfg_allow_multi_graph_ = false;
  try {
    cfg_allow_multi_graph_ = config->get_bool("/navgraph/allow_multi_graph");
  } catch (Exception &e) {} // ignored

  cfg_enable_path_execution_ = true;
  try {
    cfg_enable_path_execution_ = config->get_bool("/navgraph/path_execution");
  } catch (Exception &e) {} // ignored

  if (config->exists("/navgraph/travel_tolerance") ||
      config->exists("/navgraph/target_tolerance") ||
      config->exists("/navgraph/orientation_tolerance") ||
      config->exists("/navgraph/shortcut_tolerance"))
  {
    logger->log_error(name(), "Tolerances may no longe rbe set in the config.");
    logger->log_error(name(), "The must be set as default properties in the graph.");
    logger->log_error(name(), "Remove the following config values (move to navgraph):");
    logger->log_error(name(), "  /navgraph/travel_tolerance");
    logger->log_error(name(), "  /navgraph/target_tolerance");
    logger->log_error(name(), "  /navgraph/orientation_tolerance");
    logger->log_error(name(), "  /navgraph/shortcut_tolerance");
    throw Exception("Navgraph tolerances may no longer be set in the config");
  }

  if (cfg_enable_path_execution_) {
	  pp_nav_if_ = blackboard->open_for_writing<NavigatorInterface>("Pathplan");
	  nav_if_    = blackboard->open_for_reading<NavigatorInterface>(cfg_nav_if_id_.c_str());
	  path_if_   = blackboard->open_for_writing<NavPathInterface>("NavPath");
  }


  if (! cfg_graph_file_.empty()) {
    if (cfg_graph_file_[0] != '/') {
      cfg_graph_file_ = std::string(CONFDIR) + "/" + cfg_graph_file_;
    }
    graph_ = load_graph(cfg_graph_file_);
  } else {
    graph_ = LockPtr<NavGraph>(new NavGraph("generated"), /* recursive mutex */ true);
  }

  if (! graph_->has_default_property("travel_tolerance")) {
    throw Exception("Graph must specify travel tolerance");
  }
  if (! graph_->has_default_property("target_tolerance")) {
    throw Exception("Graph must specify target tolerance");
  }
  if (! graph_->has_default_property("orientation_tolerance")) {
    throw Exception("Graph must specify orientation tolerance");
  }
  if (! graph_->has_default_property("shortcut_tolerance")) {
    throw Exception("Graph must specify shortcut tolerance");
  }
  if (graph_->has_default_property("target_time")) {
    cfg_target_time_ = graph_->default_property_as_float("target_time");
    logger->log_info(name(), "Using target time %f from graph file", cfg_target_time_);
  }
  if (graph_->has_default_property("target_ori_time")) {
    cfg_target_time_ = graph_->default_property_as_float("target_ori_time");
    logger->log_info(name(), "Using target orientation time %f from graph file", cfg_target_ori_time_);
  }

  navgraph_aspect_inifin_.set_navgraph(graph_);
  if (cfg_log_graph_) {
    log_graph();
  }

  if (cfg_monitor_file_) {
    logger->log_info(name(), "Enabling graph file monitoring");
    try {
      fam_ = new FileAlterationMonitor();
      fam_->watch_file(cfg_graph_file_.c_str());
      fam_->add_listener(this);
    } catch (Exception &e) {
      logger->log_warn(name(), "Could not enable graph file monitoring");
      logger->log_warn(name(), e);
    }

  }

  exec_active_       = false;
  target_reached_    = false;
  target_ori_reached_= false;
  target_rotating_   = false;
  last_node_         = "";
  error_reason_      = "";
  constrained_plan_  = false;
  cmd_msgid_         = 0;
  cmd_sent_at_       = new Time(clock);
  path_planned_at_   = new Time(clock);
  target_reached_at_ = new Time(clock);
  error_at_          = new Time(clock);
#ifdef HAVE_VISUALIZATION
  visualized_at_     = new Time(clock);
  if (vt_) {
    graph_->add_change_listener(vt_);
  }
#endif

  constraint_repo_   = graph_->constraint_repo();
}

void
NavGraphThread::finalize()
{
  delete cmd_sent_at_;
  delete path_planned_at_;
  delete target_reached_at_;
  delete error_at_;
#ifdef HAVE_VISUALIZATION
  delete visualized_at_;
  if (vt_) {
    graph_->remove_change_listener(vt_);
  }
#endif
  graph_.clear();
  if (cfg_enable_path_execution_) {
	  blackboard->close(pp_nav_if_);
	  blackboard->close(nav_if_);
	  blackboard->close(path_if_);
  }
}

void
NavGraphThread::once()
{
#ifdef HAVE_VISUALIZATION
  if (vt_) {
    vt_->set_constraint_repo(constraint_repo_);
    vt_->set_graph(graph_);
  }
#endif
}

void
NavGraphThread::loop()
{
  // process messages
  bool needs_write = false;
  while (cfg_enable_path_execution_ && ! pp_nav_if_->msgq_empty()) {
    needs_write = true;

    if (pp_nav_if_->msgq_first_is<NavigatorInterface::StopMessage>()) {
      NavigatorInterface::StopMessage *msg = pp_nav_if_->msgq_first(msg);
      if (msg->msgid() == 0 || msg->msgid() == pp_nav_if_->msgid()) {
	      NavigatorInterface::StopMessage *msg = pp_nav_if_->msgq_first(msg);
	      pp_nav_if_->set_msgid(msg->id());

	      stop_motion();
	      exec_active_ = false;
      } else {
	      logger->log_warn(name(), "Received stop message for non-active command "
	                       "(got %u, running %u)", msg->msgid(), pp_nav_if_->msgid());
      }

    } else if (pp_nav_if_->msgq_first_is<NavigatorInterface::CartesianGotoMessage>()) {
      NavigatorInterface::CartesianGotoMessage *msg = pp_nav_if_->msgq_first(msg);
      logger->log_info(name(), "cartesian goto (x,y,ori) = (%f,%f,%f)",
		       msg->x(), msg->y(), msg->orientation());

      pp_nav_if_->set_msgid(msg->id());
      if (generate_plan(msg->x(), msg->y(), msg->orientation())) {
	      optimize_plan();
	      start_plan();
      } else {
	      stop_motion();
      }

    } else if (pp_nav_if_->msgq_first_is<NavigatorInterface::PlaceGotoMessage>()) {
      NavigatorInterface::PlaceGotoMessage *msg = pp_nav_if_->msgq_first(msg);
      logger->log_info(name(), "goto '%s'", msg->place());

      pp_nav_if_->set_msgid(msg->id());
      if (generate_plan(msg->place())) {
	      optimize_plan();
	      start_plan();
      } else {
	      stop_motion();
      }

    } else if (pp_nav_if_->msgq_first_is<NavigatorInterface::PlaceWithOriGotoMessage>()) {
      NavigatorInterface::PlaceWithOriGotoMessage *msg = pp_nav_if_->msgq_first(msg);
      logger->log_info(name(), "goto '%s' with ori %f", msg->place(), msg->orientation());

      pp_nav_if_->set_msgid(msg->id());
      if (generate_plan(msg->place(), msg->orientation())) {
	      optimize_plan();
	      start_plan();
      } else {
	      stop_motion();
      }
    }

    pp_nav_if_->msgq_pop();
  }

  if (cfg_monitor_file_) {
    fam_->process_events();
  }

  if (cfg_enable_path_execution_ && exec_active_) {
    // check if current was target reached
    size_t shortcut_to;

    if (! tf_listener->transform_origin(cfg_base_frame_, cfg_global_frame_, pose_)) {
      logger->log_warn(name(), "Cannot get pose info, skipping loop");

    } else if (target_reached_) {
      // reached the target, check if colli/navi/local planner is final
      nav_if_->read();
      fawkes::Time now(clock);
      if (nav_if_->is_final()) {
	pp_nav_if_->set_final(true);
	exec_active_ = false;
	needs_write = true;

      } else if (target_ori_reached_) {
	if ((now - target_reached_at_) >= target_time_) {
	  stop_motion();
	  needs_write = true;
	}

      } else if (!target_rotating_ && (now - target_reached_at_) >= target_time_) {
        if (traversal_.current().has_property("orientation")) {
          // send one last command, which will only rotate
          send_next_goal();
          target_rotating_ = true;
        } else {
          stop_motion();
          needs_write = true;
        }

      } else if (target_rotating_ && node_ori_reached()) {
	//logger->log_debug(name(), "loop(), target_rotating_, ori reached, but colli not final");
	// reset timer with new timeout value
	target_time_ = 0;
	if (traversal_.current().has_property("target_ori_time")) {
	  target_time_ = traversal_.current().property_as_float("target_ori_time");
	}
	if (target_time_ == 0)  target_time_ = cfg_target_ori_time_;

	target_ori_reached_ = true;
	target_reached_at_->stamp();
      }

    } else if (node_reached()) {
      logger->log_info(name(), "Node '%s' has been reached",
		       traversal_.current().name().c_str());
      last_node_ = traversal_.current().name();
      if (traversal_.last()) {
	target_time_ = 0;
	if (traversal_.current().has_property("target-time")) {
	  target_time_ = traversal_.current().property_as_float("target-time");
	}
	if (target_time_ == 0)  target_time_ = cfg_target_time_;

	target_reached_ = true;
	target_reached_at_->stamp();

      } else if (traversal_.next()) {
	publish_path();

        try {
          logger->log_info(name(), "Sending next goal %s after node reached",
			   traversal_.current().name().c_str());
          send_next_goal();
        } catch (Exception &e) {
          logger->log_warn(name(), "Failed to send next goal (node reached)");
          logger->log_warn(name(), e);
        }
      }

    } else if ((shortcut_to = shortcut_possible()) > 0) {
      logger->log_info(name(), "Shortcut posible, jumping from '%s' to '%s'",
		       traversal_.current().name().c_str(),
		       traversal_.path().nodes()[shortcut_to].name().c_str());

      traversal_.set_current(shortcut_to);

      if (traversal_.remaining() > 0) {
        try {
          logger->log_info(name(), "Sending next goal after taking a shortcut");
          send_next_goal();
        } catch (Exception &e) {
          logger->log_warn(name(), "Failed to send next goal (shortcut)");
          logger->log_warn(name(), e);
        }
      }

    } else {
      fawkes::Time now(clock);
      bool new_plan = false;

      if (traversal_.remaining() > 2 && (now - path_planned_at_) > cfg_replan_interval_)
      {
	*path_planned_at_ = now;
	constraint_repo_.lock();
	if (constraint_repo_->compute() || constraint_repo_->modified(/* reset */ true)) {
	  if (replan(traversal_.current(), traversal_.path().goal())) {
	    // do not optimize here, we know that we do want to travel
	    // to the first node, we are already on the way...
	    //optimize_plan();
	    start_plan();
	    new_plan = true;
	  }
	}
	constraint_repo_.unlock();
      }

      if (! new_plan && (now - cmd_sent_at_) > cfg_resend_interval_) {
        try {
          //logger->log_info(name(), "Re-sending goal");
	  send_next_goal();
        } catch (Exception &e) {
          logger->log_warn(name(), "Failed to send next goal (resending)");
          logger->log_warn(name(), e);
        }
      }
    }
  }

#ifdef HAVE_VISUALIZATION
  if (vt_) {
    fawkes::Time now(clock);
    if (now - visualized_at_ >= cfg_visual_interval_) {
      *visualized_at_ = now;
      constraint_repo_.lock();
      if (constraint_repo_->compute() || constraint_repo_->modified(/* reset */ false)) {
	vt_->wakeup();
      }
      constraint_repo_.unlock();
    }
  }
#endif

  if (cfg_enable_path_execution_ && needs_write) {
    pp_nav_if_->write();
  }
}

fawkes::LockPtr<fawkes::NavGraph>
NavGraphThread::load_graph(std::string filename)
{
  std::ifstream inf(filename);
  std::string firstword;
  inf >> firstword;
  inf.close();

  if (firstword == "%YAML") {
    logger->log_info(name(), "Loading YAML graph from %s", filename.c_str());
    return fawkes::LockPtr<NavGraph>(load_yaml_navgraph(filename, cfg_allow_multi_graph_),
                                     /* recursive mutex */ true);
  } else {
    throw Exception("Unknown graph format");
  }
}

bool
NavGraphThread::generate_plan(std::string goal_name)
{
	if (! tf_listener->transform_origin(cfg_base_frame_, cfg_global_frame_, pose_)) {
		logger->log_warn(name(),
		                 "Failed to compute pose, cannot generate plan");
		if (cfg_enable_path_execution_) {
			pp_nav_if_->set_final(true);
			pp_nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
		}
		return false;
	}

	NavGraphNode goal = graph_->node(goal_name);
	
	if (! goal.is_valid()) {
		logger->log_error(name(), "Failed to generate path to %s: goal is unknown",
		                  goal_name.c_str()); 
		if (cfg_enable_path_execution_) {
			pp_nav_if_->set_final(true);
			pp_nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
		}
    return false;
	}

	if (goal.unconnected()) {
		return generate_plan(goal.x(), goal.y(),
		                     goal.has_property("orientation")
		                     ? goal.property_as_float("orientation")
		                     : std::numeric_limits<float>::quiet_NaN(),
		                     goal.name());
	}

	NavGraphNode init =
		graph_->closest_node(pose_.getOrigin().x(), pose_.getOrigin().y());

	logger->log_debug(name(), "Starting at (%f,%f), closest node is '%s'",
	                  pose_.getOrigin().x(), pose_.getOrigin().y(), init.name().c_str());

	try {
		path_ = graph_->search_path(init, goal, /* use constraints */ true);
	} catch (Exception &e) {
		logger->log_error(name(), "Failed to generate path from (%.2f,%.2f) to %s: %s",
		                  init.x(), init.y(), goal_name.c_str(), e.what_no_backtrace());
		if (cfg_enable_path_execution_) {
			pp_nav_if_->set_final(true);
			pp_nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
		}
		return false;
	}

	if (! path_.empty()) {
		constrained_plan_ = true;
	} else {
		constrained_plan_ = false;
		logger->log_warn(name(), "Failed to generate plan, will try without constraints");
		try {
			path_ = graph_->search_path(init, goal, /* use constraints */ false);
		} catch (Exception &e) {
			if (cfg_enable_path_execution_) {
				pp_nav_if_->set_final(true);
				pp_nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
			}
			return false;
		}
	}

	if (path_.empty()) {
		logger->log_error(name(), "Failed to generate plan to travel to '%s'",
		                  goal_name.c_str());
		pp_nav_if_->set_final(true);
		pp_nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
		return false;
	}

	traversal_ = path_.traversal();
	return true;
}

bool
NavGraphThread::generate_plan(std::string goal_name, float ori)
{
	if (generate_plan(goal_name)) {

		if (! path_.empty() && std::isfinite(ori)) {
			path_.nodes_mutable().back().set_property("orientation", ori);
		}

		traversal_ = path_.traversal();
		return true;
	} else {
		if (cfg_enable_path_execution_) {
			pp_nav_if_->set_final(true);
			pp_nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
		}
	  return false;
	}
}

bool
NavGraphThread::generate_plan(float x, float y, float ori, const std::string &target_name)
{
  NavGraphNode close_to_goal = graph_->closest_node(x, y);
  if (! close_to_goal.is_valid()) {
	  logger->log_error(name(), "Cannot find closest node to target (%.2f,%.2f,%.2f) alias %s",
	                    x, y, ori, target_name.c_str());
	  return false;
  }
  if (generate_plan(close_to_goal.name())) {

	  NavGraphNode n(target_name, x, y);
	  if (std::isfinite(ori)) {
		  n.set_property("orientation", ori);
	  }
	  graph_->apply_default_properties(n);
	  path_.add_node(n);
	  traversal_ = path_.traversal();
	  return true;

  } else {
		if (cfg_enable_path_execution_) {
			pp_nav_if_->set_final(true);
			pp_nav_if_->set_error_code(NavigatorInterface::ERROR_PATH_GEN_FAIL);
		}
	  return false;
  }
}


bool
NavGraphThread::replan(const NavGraphNode &start, const NavGraphNode &goal)
{
  logger->log_debug(name(), "Starting at node '%s'", start.name().c_str());

  NavGraphNode act_goal = goal;

  NavGraphNode close_to_goal;
  if (goal.name() == "free-target") {
    close_to_goal = graph_->closest_node(goal.x(), goal.y());
    act_goal = close_to_goal;
  }

  NavGraphPath new_path =
    graph_->search_path(start, act_goal,
			  /* use constraints */ true, /* compute constraints */ false);

  if (! new_path.empty()) {
    // get cost of current plan
    NavGraphNode pose("current-pose", pose_.getOrigin().x(), pose_.getOrigin().y());
    float old_cost = graph_->cost(pose, traversal_.current()) + traversal_.remaining_cost();
    float new_cost = new_path.cost();

    if (new_cost <= old_cost * cfg_replan_factor_) {
      constrained_plan_ = true;
      path_ = new_path;
      if (goal.name() == "free-target") {
	// add free target node again
	path_.add_node(goal);
      }
      traversal_ = path_.traversal();
      logger->log_info(name(), "Executing after re-planning from '%s' to '%s', "
		       "old cost: %f  new cost: %f (%f * %f)",
		       start.name().c_str(), goal.name().c_str(),
		       old_cost, new_cost * cfg_replan_factor_, new_cost, cfg_replan_factor_);
      return true;
    } else {
      logger->log_warn(name(), "Re-planning from '%s' to '%s' resulted in "
		       "more expensive plan: %f > %f (%f * %f), keeping old",
		       start.name().c_str(), goal.name().c_str(),
		       new_cost, old_cost * cfg_replan_factor_, old_cost, cfg_replan_factor_);
      return false;
    }
  } else {
    logger->log_error(name(), "Failed to re-plan from '%s' to '%s'",
		      start.name().c_str(), goal.name().c_str());
    return false;
  }
}


/** Optimize the current plan.
 * Note that after generating a plan, the robot first needs to
 * travel to the first actual node from a free position within
 * the environment. It can happen, that this closest node lies
 * in the opposite direction of the second node, hence the robot
 * needs to "go back" first, and only then starts following
 * the path. We can optimize this by removing the first node,
 * so that the robot directly travels to the second node which
 * "lies on the way".
 */
void
NavGraphThread::optimize_plan()
{
  if (traversal_.remaining() > 1) {
    // get current position of robot in map frame
    const NavGraphPath &path = traversal_.path();
    double sqr_dist_a = ( pow(pose_.getOrigin().x() - path.nodes()[0].x(), 2) +
                          pow(pose_.getOrigin().y() - path.nodes()[0].y(), 2) );
    double sqr_dist_b = ( pow(path.nodes()[0].x() - path.nodes()[1].x(), 2) +
                          pow(path.nodes()[0].y() - path.nodes()[1].y(), 2) );
    double sqr_dist_c = ( pow(pose_.getOrigin().x() - path.nodes()[1].x(), 2) +
                          pow(pose_.getOrigin().y() - path.nodes()[1].y(), 2) );

    if (sqr_dist_a + sqr_dist_b >= sqr_dist_c){
      traversal_.next();
    }
  }
}


void
NavGraphThread::start_plan()
{
	if (! cfg_enable_path_execution_)  return;

  path_planned_at_->stamp();

  target_reached_ = false;
  target_ori_reached_ = false;
  target_rotating_ = false;
  if (traversal_.remaining() == 0) {
    exec_active_ = false;
    pp_nav_if_->set_final(true);
    pp_nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
    logger->log_warn(name(), "Cannot start empty plan.");

#ifdef HAVE_VISUALIZATION
    if (vt_) {
      vt_->reset_plan();
      visualized_at_->stamp();
    }
#endif

  } else {
    traversal_.next();

    std::string m = path_.nodes()[0].name();
    for (unsigned int i = 1; i < path_.size(); ++i) {
      m += " - " + path_.nodes()[i].name();
    }
    logger->log_info(name(), "Starting route: %s", m.c_str());
#ifdef HAVE_VISUALIZATION
    if (vt_) {
      vt_->set_traversal(traversal_);
      visualized_at_->stamp();
    }
#endif

    exec_active_ = true;

    NavGraphNode final_target = path_.goal();

    pp_nav_if_->set_error_code(NavigatorInterface::ERROR_NONE);
    pp_nav_if_->set_final(false);
    pp_nav_if_->set_dest_x(final_target.x());
    pp_nav_if_->set_dest_y(final_target.y());

    try {
      logger->log_info(name(), "Sending next goal on plan start");
      send_next_goal();
    } catch (Exception &e) {
      logger->log_warn(name(), "Failed to send next goal (start plan)");
      logger->log_warn(name(), e);
    }
  }

  publish_path();
}


void
NavGraphThread::stop_motion()
{
	if (! cfg_enable_path_execution_)  return;

  NavigatorInterface::StopMessage *stop = new NavigatorInterface::StopMessage(cmd_msgid_);
  try {
    nav_if_->msgq_enqueue(stop);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to stop motion, exception follows");
    logger->log_warn(name(), e);
  }
  last_node_ = "";
  exec_active_ = false;
  target_ori_reached_ = false;
  target_rotating_ = false;
  pp_nav_if_->set_final(true);
  traversal_.invalidate();
  cmd_msgid_ = 0;

#ifdef HAVE_VISUALIZATION
  if (vt_) {
    vt_->reset_plan();
    visualized_at_->stamp();
  }
#endif

}


void
NavGraphThread::send_next_goal()
{
	if (! cfg_enable_path_execution_)  return;

	bool stop_at_target   = false;
  bool orient_at_target = false;

  if (! traversal_.running()) {
    throw Exception("Cannot send next goal if plan is empty");
  }

  const NavGraphNode &next_target = traversal_.current();

  float ori = NAN;
  if ( traversal_.last() ) {
    stop_at_target = true;

    if ( next_target.has_property("orientation") ) {
      orient_at_target  = true;

      // take the given orientation for the final node
      ori = next_target.property_as_float("orientation");
    }

  } else {
    // set direction facing from next_target (what is the current point
    // to drive to) to next point to drive to.  So orientation is the
    // direction from next_target to the target after that
    const NavGraphNode &next_next_target = traversal_.peek_next();

    ori = atan2f( next_next_target.y() - next_target.y(),
                  next_next_target.x() - next_target.x());

  }

  // get target position in base frame
  tf::Stamped<tf::Pose> tpose;
  tf::Stamped<tf::Pose>
    tposeglob(tf::Transform(tf::create_quaternion_from_yaw(ori),
			    tf::Vector3(next_target.x(), next_target.y(), 0)),
	      Time(0,0), cfg_global_frame_);
  try {
    tf_listener->transform_pose(cfg_base_frame_, tposeglob, tpose);
  } catch (Exception &e) {
    logger->log_warn(name(),
		     "Failed to compute pose, cannot generate plan: %s", e.what());
    throw;
  }

  if( target_reached_ ) {
    // no need for traveling anymore, just rotating
    tpose.setOrigin(tf::Vector3(0.f, 0.f, 0.f));
  }

  NavigatorInterface::CartesianGotoMessage *gotomsg =
    new NavigatorInterface::CartesianGotoMessage(tpose.getOrigin().x(),
                                                 tpose.getOrigin().y(),
                                                 tf::get_yaw(tpose.getRotation()));

  NavigatorInterface::SetStopAtTargetMessage* stop_at_target_msg      = new NavigatorInterface::SetStopAtTargetMessage(stop_at_target);
  NavigatorInterface::SetOrientationModeMessage* orient_mode_msg;
  if ( orient_at_target ) {
    orient_mode_msg = new NavigatorInterface::SetOrientationModeMessage(
        fawkes::NavigatorInterface::OrientationMode::OrientAtTarget );
  } else {
    orient_mode_msg = new NavigatorInterface::SetOrientationModeMessage(
            fawkes::NavigatorInterface::OrientationMode::OrientDuringTravel );
  }

  try {
#ifdef HAVE_VISUALIZATION
    if (vt_)  vt_->set_current_edge(last_node_, next_target.name());
#endif

    if (! nav_if_->has_writer()) {
      throw Exception("No writer for navigator interface");
    }

    nav_if_->msgq_enqueue(stop_at_target_msg);
    nav_if_->msgq_enqueue(orient_mode_msg);

    logger->log_debug(name(), "Sending goto(x=%f,y=%f,ori=%f) for node '%s'",
		      tpose.getOrigin().x(), tpose.getOrigin().y(),
		      tf::get_yaw(tpose.getRotation()), next_target.name().c_str());


    gotomsg->ref();
    nav_if_->msgq_enqueue(gotomsg);
    cmd_msgid_ = gotomsg->id();
    gotomsg->unref();
    cmd_sent_at_->stamp();

    error_at_->stamp();
    error_reason_ = "";

  } catch (Exception &e) {
    if (cfg_abort_on_error_) {
      logger->log_warn(name(), "Failed to send cartesian goto for "
		       "next goal, exception follows");
      logger->log_warn(name(), e);
      exec_active_ = false;
      pp_nav_if_->set_final(true);
      pp_nav_if_->set_error_code(NavigatorInterface::ERROR_OBSTRUCTION);
      pp_nav_if_->write();
#ifdef HAVE_VISUALIZATION
      if (vt_)  vt_->reset_plan();
#endif
    } else {
      fawkes::Time now(clock);
      if (error_reason_ != e.what_no_backtrace() || (now - error_at_) > 4.0) {
	error_reason_ = e.what_no_backtrace();
	*error_at_ = now;
	logger->log_warn(name(), "Failed to send cartesian goto for "
			 "next goal, exception follows");
	logger->log_warn(name(), e);
	logger->log_warn(name(), "*** NOT aborting goal (as per config)");
      }
    }
  }
}

/** Check if current node has been reached.
 * Compares the distance to the node to defined tolerances.
 */
bool
NavGraphThread::node_reached()
{
  if (! traversal_) {
    logger->log_error(name(), "Cannot check node reached if no traversal given");
    return true;
  }

  if (! traversal_.running()) {
    logger->log_error(name(), "Cannot check node reached if no traversal running");
    return true;
  }

  const NavGraphNode &cur_target = traversal_.current();

  // get distance to current target in map frame
  float dist = sqrt(pow(pose_.getOrigin().x() - cur_target.x(), 2) +
		    pow( pose_.getOrigin().y() - cur_target.y(), 2));

  float tolerance = cur_target.property_as_float("travel_tolerance");
  // use a different tolerance for the final node
  if (traversal_.last()) {
    tolerance = cur_target.property_as_float("target_tolerance");
    //return (dist <= tolerance) && node_ori_reached(cur_target);
  }

  // can be no or invalid tolerance, be very generous
  if (tolerance == 0.) {
    logger->log_warn(name(), "Invalid tolerance for node %s, using 1.0",
		     cur_target.name().c_str());
    tolerance = 1.0;
  }

  return (dist <= tolerance);
}


/** Check if orientation of current node has been reached.
 * Compares the angular distance to the targeted orientation
 * to the defined angular tolerances.
 */
bool
NavGraphThread::node_ori_reached()
{
  if (! traversal_) {
    logger->log_error(name(), "Cannot check node reached if no traversal given");
    return true;
  }

  if (! traversal_.running()) {
    logger->log_error(name(), "Cannot check node reached if no traversal running");
    return true;
  }

  const NavGraphNode &cur_target = traversal_.current();
  return node_ori_reached(cur_target);
}


/** Check if orientation of a given node has been reached.
 * Compares the angular distance to the targeted orientation
 * to the defined angular tolerances.
 */
bool
NavGraphThread::node_ori_reached(const NavGraphNode &node)
{
  if (node.has_property("orientation")) {
    float ori_tolerance = node.property_as_float("orientation_tolerance");
    float ori_diff  =
      angle_distance( normalize_rad(tf::get_yaw(pose_.getRotation())),
			    normalize_rad(node.property_as_float("orientation")));

    //logger->log_info(name(), "Ori=%f Rot=%f Diff=%f Tol=%f Dist=%f Tol=%f", cur_target.property_as_float("orientation"), tf::get_yaw(pose_.getRotation() ), ori_diff, ori_tolerance, dist, tolerance);
    return (ori_diff <= ori_tolerance);

  } else {
    return true;
  }
}


size_t
NavGraphThread::shortcut_possible()
{
  if (!traversal_ || traversal_.remaining() < 1) {
    logger->log_debug(name(), "Cannot shortcut if no path nodes remaining");
    return 0;
  }

  for (size_t i = traversal_.path().size() - 1; i > traversal_.current_index(); --i) {
    const NavGraphNode &node = traversal_.path().nodes()[i];

    float dist = sqrt(pow(pose_.getOrigin().x() - node.x(), 2) +
		      pow(pose_.getOrigin().y() - node.y(), 2));

    float tolerance = node.property_as_float("shortcut_tolerance");

    if (tolerance == 0.0)  return 0;
    if (dist <= tolerance) return i;
  }

  return 0;
}


void
NavGraphThread::fam_event(const char *filename, unsigned int mask)
{
  // The file will be ignored from now onwards, re-register
  if (mask & FAM_IGNORED) {
    fam_->watch_file(cfg_graph_file_.c_str());
  }

  if (mask & (FAM_MODIFY | FAM_IGNORED)) {
    logger->log_info(name(), "Graph changed on disk, reloading");

    try {
      LockPtr<NavGraph> new_graph = load_graph(cfg_graph_file_);
      **graph_ = **new_graph;
    } catch (Exception &e) {
      logger->log_warn(name(), "Loading new graph failed, exception follows");
      logger->log_warn(name(), e);
      return;
    }

#ifdef HAVE_VISUALIZATION
    if (vt_) {
      vt_->set_graph(graph_);
      visualized_at_->stamp();
    }
#endif

    if (exec_active_) {
      // store the goal and restart it after the graph has been reloaded

      stop_motion();
      NavGraphNode goal = path_.goal();

      bool gen_ok = false;
      if (goal.name() == "free-target") {
	      gen_ok = generate_plan(goal.x(), goal.y(), goal.property_as_float("orientation"));
      } else {
	      gen_ok = generate_plan(goal.name());
      }

      if (gen_ok) {
	      optimize_plan();
	      start_plan();
      } else {
	      stop_motion();
      }
    }
  }
}


void
NavGraphThread::log_graph()
{
  const std::vector<NavGraphNode> & nodes = graph_->nodes();
  std::vector<NavGraphNode>::const_iterator n;
  for (n = nodes.begin(); n != nodes.end(); ++n) {
    logger->log_info(name(), "Node %s @ (%f,%f)%s",
		     n->name().c_str(), n->x(), n->y(),
		     n->unconnected() ? " UNCONNECTED" : "");

    const std::map<std::string, std::string> &props = n->properties();
    std::map<std::string, std::string>::const_iterator p;
    for (p = props.begin(); p != props.end(); ++p) {
      logger->log_info(name(), "  - %s: %s", p->first.c_str(), p->second.c_str());
    }
  }

  std::vector<NavGraphEdge> edges = graph_->edges();
  std::vector<NavGraphEdge>::iterator e;
  for (e = edges.begin(); e != edges.end(); ++e) {
    logger->log_info(name(), "Edge %10s --%s %s",
		     e->from().c_str(), e->is_directed() ? ">" : "-", e->to().c_str());

    const std::map<std::string, std::string> &props = e->properties();
    std::map<std::string, std::string>::const_iterator p;
    for (p = props.begin(); p != props.end(); ++p) {
      logger->log_info(name(), "  - %s: %s", p->first.c_str(), p->second.c_str());
    }
  }
}

void
NavGraphThread::publish_path()
{
	if (! cfg_enable_path_execution_)  return;

  std::vector<std::string> vpath(40, "");

  if (traversal_) {
    size_t ind = 0;
    size_t r = traversal_.running() ? traversal_.current_index() : traversal_.remaining();
    for (; r < traversal_.path().size(); ++r) {
      vpath[ind++] = traversal_.path().nodes()[r].name();
    }
  }

  path_if_->set_path_node_1(vpath[0].c_str());
  path_if_->set_path_node_2(vpath[1].c_str());
  path_if_->set_path_node_3(vpath[2].c_str());
  path_if_->set_path_node_4(vpath[3].c_str());
  path_if_->set_path_node_5(vpath[4].c_str());
  path_if_->set_path_node_6(vpath[5].c_str());
  path_if_->set_path_node_7(vpath[6].c_str());
  path_if_->set_path_node_8(vpath[7].c_str());
  path_if_->set_path_node_9(vpath[8].c_str());
  path_if_->set_path_node_10(vpath[9].c_str());
  path_if_->set_path_node_11(vpath[10].c_str());
  path_if_->set_path_node_12(vpath[11].c_str());
  path_if_->set_path_node_13(vpath[12].c_str());
  path_if_->set_path_node_14(vpath[13].c_str());
  path_if_->set_path_node_15(vpath[14].c_str());
  path_if_->set_path_node_16(vpath[15].c_str());
  path_if_->set_path_node_17(vpath[16].c_str());
  path_if_->set_path_node_18(vpath[17].c_str());
  path_if_->set_path_node_19(vpath[18].c_str());
  path_if_->set_path_node_20(vpath[19].c_str());
  path_if_->set_path_node_21(vpath[20].c_str());
  path_if_->set_path_node_22(vpath[21].c_str());
  path_if_->set_path_node_23(vpath[22].c_str());
  path_if_->set_path_node_24(vpath[23].c_str());
  path_if_->set_path_node_25(vpath[24].c_str());
  path_if_->set_path_node_26(vpath[25].c_str());
  path_if_->set_path_node_27(vpath[26].c_str());
  path_if_->set_path_node_28(vpath[27].c_str());
  path_if_->set_path_node_29(vpath[28].c_str());
  path_if_->set_path_node_30(vpath[29].c_str());
  path_if_->set_path_node_31(vpath[30].c_str());
  path_if_->set_path_node_32(vpath[31].c_str());
  path_if_->set_path_node_33(vpath[32].c_str());
  path_if_->set_path_node_34(vpath[33].c_str());
  path_if_->set_path_node_35(vpath[34].c_str());
  path_if_->set_path_node_36(vpath[35].c_str());
  path_if_->set_path_node_37(vpath[36].c_str());
  path_if_->set_path_node_38(vpath[37].c_str());
  path_if_->set_path_node_39(vpath[38].c_str());
  path_if_->set_path_node_40(vpath[39].c_str());
  path_if_->set_path_length(traversal_.remaining());
  path_if_->write();
}
