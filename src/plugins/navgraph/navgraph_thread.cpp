/***************************************************************************
 *  navgraph_thread.cpp - Graph-based global path planning
 *
 *  Created: Tue Sep 18 16:00:34 2012
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

#include "navgraph_thread.h"

#include <utils/search/astar.h>
#include <tf/utils.h>

#include "search_state.h"

using namespace fawkes;

/** @class NavGraphThread "navgraph_thread.h"
 * Thread to perform graph-based path planning.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphThread::NavGraphThread()
  : Thread("NavGraphThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
{
#ifdef HAVE_VISUALIZATION
  vt_ = NULL;
#endif
}

#ifdef HAVE_VISUALIZATION
/** Constructor. */
NavGraphThread::NavGraphThread(NavGraphVisualizationThread *vt)
  : Thread("NavGraphThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT)
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
  cfg_graph_file_      = config->get_string("/plugins/navgraph/graph_file");
  cfg_base_frame_      = config->get_string("/plugins/navgraph/base_frame");
  cfg_global_frame_    = config->get_string("/plugins/navgraph/global_frame");
  cfg_nav_if_id_       = config->get_string("/plugins/navgraph/navigator_interface_id");
  cfg_tolerance_       = config->get_float("/plugins/navgraph/tolerance");
  cfg_resend_interval_ = config->get_float("/plugins/navgraph/resend_interval");


  pp_nav_if_ = blackboard->open_for_writing<NavigatorInterface>("Pathplan");
  nav_if_    = blackboard->open_for_reading<NavigatorInterface>(cfg_nav_if_id_.c_str());

  if (cfg_graph_file_[0] != '/') {
    cfg_graph_file_ = std::string(CONFDIR) + "/" + cfg_graph_file_;
  }

  graph_ = load_rcsoft_graph(cfg_graph_file_);
  astar_     = new AStar();

  exec_active_ = false;
  last_node_   = "";
  cmd_sent_at_ = new Time(clock);
}

void
NavGraphThread::finalize()
{
  delete cmd_sent_at_;
  delete astar_;
  delete graph_;
  blackboard->close(pp_nav_if_);
  blackboard->close(nav_if_);
}

void
NavGraphThread::loop()
{
  // process messages
  bool needs_write = false;
  while (! pp_nav_if_->msgq_empty()) {
    needs_write = true;

    if (pp_nav_if_->msgq_first_is<NavigatorInterface::StopMessage>()) {

      stop_motion();
      exec_active_ = false;

    } else if (pp_nav_if_->msgq_first_is<NavigatorInterface::CartesianGotoMessage>()) {
      NavigatorInterface::CartesianGotoMessage *msg = pp_nav_if_->msgq_first(msg);
      logger->log_info(name(), "cartesian goto (x,y,ori) = (%f,%f,%f)",
		       msg->x(), msg->y(), msg->orientation());

      pp_nav_if_->set_msgid(msg->id());
      generate_plan(msg->x(), msg->y(), msg->orientation());
      start_plan();

    } else if (pp_nav_if_->msgq_first_is<NavigatorInterface::PlaceGotoMessage>()) {
      NavigatorInterface::PlaceGotoMessage *msg = pp_nav_if_->msgq_first(msg);
      logger->log_info(name(), "goto '%s'", msg->place());

      pp_nav_if_->set_msgid(msg->id());
      generate_plan(msg->place());
      start_plan();
    }

    pp_nav_if_->msgq_pop();
  }

  if (exec_active_) {
    // check if current was target reached
    if (node_reached()) {
      logger->log_info(name(), "Node '%s' has been reached", plan_[0].name().c_str());
      last_node_ = plan_[0].name();
      plan_.erase(plan_.begin());
      if (plan_.empty()) {
	stop_motion();
	pp_nav_if_->set_final(true);
	needs_write = true;
      } else {
	send_next_goal();
      }
    } else {
      fawkes::Time now(clock);
      if ((now - cmd_sent_at_) > cfg_resend_interval_) {
	send_next_goal();
      }
    }
  }

  if (needs_write) {
    pp_nav_if_->write();
  }
}



void
NavGraphThread::generate_plan(std::string goal_name)
{
  // get current position of robot in map frame
  tf::Stamped<tf::Pose> pose;
  tf::Stamped<tf::Pose> ident = tf::ident(cfg_base_frame_);
  try {
    tf_listener->transform_pose(cfg_global_frame_, ident, pose);
  } catch (Exception &e) {
    logger->log_warn(name(),
		     "Failed to compute pose, cannot generate plan", e.what());
    throw;
  }


  TopologicalMapNode init =
    graph_->closest_node(pose.getOrigin().x(), pose.getOrigin().y());
  TopologicalMapNode goal = graph_->node(goal_name);


  logger->log_debug(name(), "Starting at (%f,%f), closest node is '%s'",
		    pose.getOrigin().x(), pose.getOrigin().y(), init.name().c_str());

  plan_.clear();
  
  NavGraphSearchState *initial_state =
    new NavGraphSearchState(init, goal, 0, NULL, graph_);

  std::vector<AStarState *> a_star_solution =  astar_->solve(initial_state);

  NavGraphSearchState *solstate;
  for (unsigned int i = 0; i < a_star_solution.size(); ++i ) {
    solstate = dynamic_cast<NavGraphSearchState *>(a_star_solution[i]);
    plan_.push_back(solstate->node());
  }

  if (plan_.empty()) {
    logger->log_error(name(), "Failed to generate plan to travel to '%s'",
		      goal_name.c_str());
  } else {
    std::string m = "Route: " + plan_[0].name();
    for (unsigned int i = 1; i < plan_.size(); ++i) {
      m += " - " + plan_[i].name();
    }
    logger->log_info(name(), "%s", m.c_str());
#ifdef HAVE_VISUALIZATION
    if (vt_)  vt_->set_plan(plan_);
#endif
  }

}

void
NavGraphThread::generate_plan(float x, float y, float ori)
{
  TopologicalMapNode close_to_goal = graph_->closest_node(x, y);
  
  generate_plan(close_to_goal.name());

  TopologicalMapNode n("free-target", x, y);
  n.set_property("ori", ori);
  plan_.push_back(n);

#ifdef HAVE_VISUALIZATION
  if (vt_)  vt_->set_plan(plan_);
#endif
}


void
NavGraphThread::start_plan()
{
  if (plan_.empty()) {
    exec_active_ = false;
    pp_nav_if_->set_final(true);
    pp_nav_if_->set_error_code(NavigatorInterface::ERROR_UNKNOWN_PLACE);
    logger->log_warn(name(), "Cannot start empty plan.");
  } else {    
    exec_active_ = true;

    TopologicalMapNode &final_target = plan_.back();

    pp_nav_if_->set_error_code(NavigatorInterface::ERROR_NONE);
    pp_nav_if_->set_final(false);
    pp_nav_if_->set_dest_x(final_target.x());
    pp_nav_if_->set_dest_y(final_target.y());

    send_next_goal();
  }
}


void
NavGraphThread::stop_motion()
{
  NavigatorInterface::StopMessage *stop = new NavigatorInterface::StopMessage();
  try {
    nav_if_->msgq_enqueue(stop);
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to stop motion, exception follows");
    logger->log_warn(name(), e);
  }
  last_node_ = "";
  exec_active_ = false;
  pp_nav_if_->set_final(true);

#ifdef HAVE_VISUALIZATION
  if (vt_)  vt_->reset_plan();
#endif

}


void
NavGraphThread::send_next_goal()
{
  if (plan_.empty()) {
    throw Exception("Cannot send next goal if plan is empty");
  }

  TopologicalMapNode &next_target = plan_.front();

  // get current position of robot in map frame
  tf::Stamped<tf::Pose> tpose;
  tf::Stamped<tf::Pose>
    tposeglob(tf::Transform(tf::Quaternion(0, 0, 0, 1),
			    tf::Vector3(next_target.x(), next_target.y(), 0)),
	  Time(0,0), cfg_global_frame_);
  try {
    tf_listener->transform_pose(cfg_base_frame_, tposeglob, tpose);
  } catch (Exception &e) {
    logger->log_warn(name(),
		     "Failed to compute pose, cannot generate plan", e.what());
    throw;
  }

  logger->log_info(name(), "Sending goto(x=%f,y=%f,ori=%f) for node '%s'",
                  tpose.getOrigin().x(), tpose.getOrigin().y(),
                  tf::get_yaw(tpose.getRotation()), next_target.name().c_str());

  NavigatorInterface::CartesianGotoMessage *gotomsg =
    new NavigatorInterface::CartesianGotoMessage(tpose.getOrigin().x(),
						 tpose.getOrigin().y(),
						 tf::get_yaw(tpose.getRotation()));
  try {
    nav_if_->msgq_enqueue(gotomsg);
    cmd_sent_at_->stamp();

#ifdef HAVE_VISUALIZATION
    if (vt_)  vt_->set_current_edge(last_node_, next_target.name());
#endif
  } catch (Exception &e) {
    logger->log_warn(name(), "Failed to send cartesian goto for next goal, exception follows");
    logger->log_warn(name(), e);
#ifdef HAVE_VISUALIZATION
    if (vt_)  vt_->reset_plan();
#endif
  }
}


bool
NavGraphThread::node_reached()
{
  if (plan_.empty()) {
    throw Exception("Cannot check node reached if plan is empty");
  }

  TopologicalMapNode &cur_target = plan_.front();

  // get current position of robot in map frame
  tf::Stamped<tf::Pose> pose;
  tf::Stamped<tf::Pose> ident = tf::ident(cfg_base_frame_);
  try {
    tf_listener->transform_pose(cfg_global_frame_, ident, pose);
  } catch (Exception &e) {
    logger->log_warn(name(),
		     "Failed to compute pose, cannot generate plan", e.what());
    throw;
  }

  float dist = sqrt(pow(pose.getOrigin().x() - cur_target.x(), 2) +
		    pow(pose.getOrigin().y() - cur_target.y(), 2));

  return (dist <= cfg_tolerance_);
}
