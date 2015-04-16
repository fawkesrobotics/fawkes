/***************************************************************************
 *  navgraph_interactive_thread.cpp - Interactive navgraph editing
 *
 *  Created: Thu Jan 15 16:26:40 2015
 *  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

#include "navgraph_interactive_thread.h"

#include <navgraph/yaml_navgraph.h>
#include <tf/types.h>

#include <interactive_markers/interactive_marker_server.h>

using namespace fawkes;
using namespace interactive_markers;

/** @class NavGraphInteractiveThread "navgraph_interactive_thread.h"
 * Thread to perform graph-based path planning.
 * @author Tim Niemueller
 */

/** Constructor. */
NavGraphInteractiveThread::NavGraphInteractiveThread()
  : Thread("NavGraphInteractiveThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
NavGraphInteractiveThread::~NavGraphInteractiveThread()
{
}


void
NavGraphInteractiveThread::process_node_ori_feedback
  (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
   const NodeMenu &menu, visualization_msgs::InteractiveMarker &int_marker)
{
  const std::shared_ptr<MenuHandler> &handler = menu.handler;
  MenuHandler::EntryHandle entry_handle = (MenuHandler::EntryHandle)feedback->menu_entry_id;
  MenuHandler::CheckState check_state;
  if (handler->getCheckState(entry_handle, check_state)) {
    if (check_state == MenuHandler::UNCHECKED) {
      visualization_msgs::InteractiveMarkerControl ori_control;
      ori_control.name = "rot_z";
      ori_control.orientation.w = 1;
      ori_control.orientation.x = 0;
      ori_control.orientation.y = 1;
      ori_control.orientation.z = 0;
      ori_control.interaction_mode =
	visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(ori_control);
      handler->setCheckState(entry_handle, MenuHandler::CHECKED);

      auto control =
	std::find_if(int_marker.controls.begin(), int_marker.controls.end(),
		     [](const visualization_msgs::InteractiveMarkerControl &c)->bool {
		       return c.name == "menu";
		     });
      if (control != int_marker.controls.end() && ! control->markers.empty()) {
	visualization_msgs::Marker &box_marker = control->markers[0];
	box_marker.type = visualization_msgs::Marker::ARROW;
	box_marker.points.clear();
	geometry_msgs::Point p1, p2;
	p1.x = p1.y = p1.z = 0.;
	p2.x = 0.2;
	p2.y = p2.z = 0.;
	box_marker.points.push_back(p1);
	box_marker.points.push_back(p2);
	box_marker.scale.x = 0.35;
	box_marker.scale.y = 0.35;
	box_marker.scale.z = 0.2;
      }
    } else {
      int_marker.controls.erase(
	std::remove_if(int_marker.controls.begin(), int_marker.controls.end(),
		       [](const visualization_msgs::InteractiveMarkerControl &c)->bool {
			 return c.name == "rot_z";
		       }),
	int_marker.controls.end());
      handler->setCheckState(entry_handle, MenuHandler::UNCHECKED);

      auto control =
	std::find_if(int_marker.controls.begin(), int_marker.controls.end(),
		     [](const visualization_msgs::InteractiveMarkerControl &c)->bool {
		       return c.name == "menu";
		     });
      if (control != int_marker.controls.end() && ! control->markers.empty()) {
	visualization_msgs::Marker &box_marker = control->markers[0];
	box_marker.points.clear();
	box_marker.type = visualization_msgs::Marker::SPHERE;
	box_marker.scale.x = 0.25;
	box_marker.scale.y = 0.25;
	box_marker.scale.z = 0.25;
      }
    }
    server_->insert(int_marker, boost::bind(&NavGraphInteractiveThread::process_node_feedback, this, _1));
    server_->applyChanges();
    handler->reApply(*server_);
  } else {
    logger->log_warn(name(), "Got menu feedback for %s/%s, but check state cannot be retrieved",
		     feedback->marker_name.c_str(), feedback->control_name.c_str());
  }
}

void
NavGraphInteractiveThread::process_node_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      logger->log_debug(name(), "%s/%s: menu item %u clicked",
			feedback->marker_name.c_str(), feedback->control_name.c_str(),
			feedback->menu_entry_id);
      {
	visualization_msgs::InteractiveMarker int_marker;
	if (server_->get(feedback->marker_name, int_marker)) {
	  if (node_menus_.find(int_marker.name) != node_menus_.end()) {
	    NodeMenu &menu = node_menus_[int_marker.name];
	    if (feedback->menu_entry_id == menu.ori_handle) {
	      process_node_ori_feedback(feedback, menu, int_marker);
	    } else if (feedback->menu_entry_id == menu.goto_handle) {
	      if (nav_if_->has_writer()) {
		NavigatorInterface::PlaceGotoMessage *gotomsg =
		  new NavigatorInterface::PlaceGotoMessage(int_marker.name.c_str());
		nav_if_->msgq_enqueue(gotomsg);
	      } else {
		logger->log_error(name(), "Cannot goto %s, no writer for interface",
				  int_marker.name.c_str());
	      }
	    } else if (feedback->menu_entry_id == menu.remove_handle) {
	      navgraph.lock();
	      navgraph->remove_node(feedback->marker_name);
	      navgraph->calc_reachability(true);
	      navgraph.unlock();
	      server_->erase(feedback->marker_name);
	      server_->applyChanges();
	    } else if (menu.undir_connect_nodes.find(feedback->menu_entry_id) != menu.undir_connect_nodes.end()) {
	      std::string to_node = menu.undir_connect_nodes[feedback->menu_entry_id];
	      NavGraphEdge edge(int_marker.name, to_node);
	      navgraph.lock();
	      try {
		navgraph->add_edge(edge);
		navgraph->calc_reachability(true);
	      } catch (Exception &e) {
		navgraph.unlock();
		logger->log_error(name(), "Failed to insert edge %s--%s as requested, exception follows",
				  int_marker.name.c_str(), to_node.c_str());
		logger->log_error(name(), e);
	      }
	      server_->erase(int_marker.name);
	      add_node(navgraph->node(int_marker.name), *navgraph);
	      server_->erase(to_node);
	      add_node(navgraph->node(to_node), *navgraph);
	      navgraph.unlock();
	      server_->applyChanges();

	    } else if (menu.dir_connect_nodes.find(feedback->menu_entry_id) != menu.dir_connect_nodes.end()) {
	      NavGraphEdge edge(int_marker.name,
				menu.dir_connect_nodes[feedback->menu_entry_id]);
	      edge.set_directed(true);
	      navgraph.lock();
	      try {
		navgraph->add_edge(edge);
		navgraph->calc_reachability(true);
	      } catch (Exception &e) {
		navgraph.unlock();
		logger->log_error(name(), "Failed to insert edge %s->%s as requested, "
				  "exception follows", int_marker.name.c_str(),
				  menu.dir_connect_nodes[feedback->menu_entry_id].c_str());
		logger->log_error(name(), e);
	      }
	      server_->erase(int_marker.name);
	      add_node(navgraph->node(int_marker.name), *navgraph);
	      navgraph.unlock();
	      server_->applyChanges();

	    } else if (menu.disconnect_nodes.find(feedback->menu_entry_id) != menu.disconnect_nodes.end()) {
	      navgraph.lock();
	      std::string to_node = menu.disconnect_nodes[feedback->menu_entry_id];
	      NavGraphEdge edge = navgraph->edge(feedback->marker_name, to_node);
	      if (! edge) {
		logger->log_error(name(), "Failed to find edge %s--%s",
				  feedback->marker_name.c_str(), to_node.c_str());
	      }
	      navgraph->remove_edge(edge);
	      navgraph->calc_reachability(true);
	      server_->erase(feedback->marker_name);
	      add_node(navgraph->node(feedback->marker_name), *navgraph);
	      if (! edge.is_directed()) {
		server_->erase(to_node);
		add_node(navgraph->node(to_node), *navgraph);
	      }
	      navgraph.unlock();
	      server_->applyChanges();
	    } else {
	      logger->log_warn(name(), "Got menu feedback for %s/%s, but marker not known",
			       feedback->marker_name.c_str(), feedback->control_name.c_str());
	    }
	    
	  } else {
	    logger->log_warn(name(), "Got feedback for %s/%s, but marker not known",
			     feedback->marker_name.c_str(), feedback->control_name.c_str());
	  }
	}
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      if (feedback->header.frame_id == cfg_global_frame_) {
	navgraph.lock();
	NavGraphNode node = navgraph->node(feedback->marker_name);
	if (node) {

	  if (feedback->control_name == "rot_z") {
	    // orientation update
	    tf::Quaternion q(feedback->pose.orientation.x, feedback->pose.orientation.y,
			     feedback->pose.orientation.z, feedback->pose.orientation.w);
	    node.set_property(navgraph::PROP_ORIENTATION, (float)tf::get_yaw(q));
	  } else {
	    // position update
	    node.set_x(feedback->pose.position.x);
	    node.set_y(feedback->pose.position.y);
	  }
	  navgraph->update_node(node);
	} else {
	  logger->log_warn(name(), "Position update for %s, but not unknown",
			   feedback->marker_name.c_str());
	}
	navgraph.unlock();
	navgraph->notify_of_change();
      } else {
	logger->log_warn(name(), "Interactive marker feedback in non-global frame %s, ignoring",
			 feedback->header.frame_id.c_str());
      }
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      break;
  }

  server_->applyChanges();
}


void
NavGraphInteractiveThread::process_graph_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT) {
    if (feedback->menu_entry_id == graph_menu_.stop_handle) {
      if (nav_if_->has_writer()) {
	NavigatorInterface::StopMessage *stop = new NavigatorInterface::StopMessage();
	nav_if_->msgq_enqueue(stop);
      } else {
	logger->log_error(name(), "Cannot stop, no writer for interface");
      }
    } else if (feedback->menu_entry_id == graph_menu_.add_handle) {
      navgraph.lock();
      for (unsigned int i = 0; i < 1000; ++i) {
	std::string name = NavGraph::format_name("N%u", i);
	if (! navgraph->node_exists(name)) {
	  // valid new node name, create node
	  NavGraphNode node(name, 0., 0.);
	  navgraph->add_node(node);
	  add_node(node, *navgraph);
	  navgraph->calc_reachability(true);
	  server_->applyChanges();
	  break;
	}
      }
      navgraph.unlock();
    } else if (feedback->menu_entry_id == graph_menu_.save_handle) {
      navgraph.lock();
      save_yaml_navgraph(cfg_save_filename_, *navgraph);
      navgraph.unlock();
    }
  }
}

void
NavGraphInteractiveThread::init()
{
  cfg_global_frame_    = config->get_string("/navgraph/global_frame");
  cfg_save_filename_   = config->get_string("/navgraph/interactive/out-file");

  // create an interactive marker server on the topic namespace simple_marker
  server_ = new interactive_markers::InteractiveMarkerServer("navgraph_interactive");

  navgraph.lock();
  add_graph(*navgraph);

  const std::vector<NavGraphNode> &nodes = navgraph->nodes();
  for (const NavGraphNode &node : nodes) {
    add_node(node, *navgraph);
  }
  navgraph.unlock();

  // 'commit' changes and send to all clients
  server_->applyChanges();

  nav_if_ = blackboard->open_for_reading<NavigatorInterface>("Pathplan");
}

void
NavGraphInteractiveThread::finalize()
{
  node_menus_.clear();
  graph_menu_handler_.reset();
  delete server_;
  blackboard->close(nav_if_);
}

void
NavGraphInteractiveThread::loop()
{
}


void
NavGraphInteractiveThread::add_node(const NavGraphNode &node, NavGraph *navgraph)
{
  const bool has_ori = node.has_property(navgraph::PROP_ORIENTATION);
  const tf::Quaternion ori_q = has_ori
    ? tf::create_quaternion_from_yaw(node.property_as_float(navgraph::PROP_ORIENTATION))
    : tf::Quaternion(0,0,0,1);

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = cfg_global_frame_;
  int_marker.name = node.name();
  int_marker.description = ""; //node.name();
  int_marker.scale = 0.5;

  int_marker.pose.position.x = node.x();
  int_marker.pose.position.y = node.y();
  int_marker.pose.position.z = 0.;
  if (has_ori) {
    int_marker.pose.orientation.x = ori_q[0];
    int_marker.pose.orientation.y = ori_q[1];
    int_marker.pose.orientation.z = ori_q[2];
    int_marker.pose.orientation.w = ori_q[3];
  } else {
    int_marker.pose.orientation.x = int_marker.pose.orientation.y = int_marker.pose.orientation.z = 0.;
    int_marker.pose.orientation.w = 1.;
  }

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  if (has_ori) {
    box_marker.type = visualization_msgs::Marker::ARROW;
    geometry_msgs::Point p1, p2;
    p1.x = p1.y = p1.z = 0.;
    p2.x = 0.2;
    p2.y = p2.z = 0.;
    box_marker.points.push_back(p1);
    box_marker.points.push_back(p2);
    box_marker.scale.x = 0.35;
    box_marker.scale.y = 0.35;
    box_marker.scale.z = 0.2;

  } else {
    box_marker.type = visualization_msgs::Marker::SPHERE;
    box_marker.scale.x = 0.25;
    box_marker.scale.y = 0.25;
    box_marker.scale.z = 0.25;
  }
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);
  box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  box_control.description="Options";
  box_control.name="menu";
  int_marker.controls.push_back(box_control);

  NodeMenu menu;
  menu.handler = std::shared_ptr<MenuHandler>(new MenuHandler());
  menu.ori_handle =
    menu.handler->insert("Orientation", boost::bind(&NavGraphInteractiveThread::process_node_feedback, this, _1));
  menu.goto_handle =
    menu.handler->insert("Go to", boost::bind(&NavGraphInteractiveThread::process_node_feedback, this, _1));
  menu.handler->setCheckState(menu.ori_handle, MenuHandler::UNCHECKED);
  const std::vector<NavGraphNode> &nodes = navgraph->nodes();
  const std::vector<NavGraphEdge> &edges = navgraph->edges();
  MenuHandler::EntryHandle connect_undir_menu_handle = menu.handler->insert("Connect with");
  MenuHandler::EntryHandle connect_dir_menu_handle = menu.handler->insert("Connect directed");
  MenuHandler::EntryHandle disconnect_menu_handle = menu.handler->insert("Disconnect from");
  std::for_each(nodes.begin(), nodes.end(),
		[&, this](const NavGraphNode &n)->void {
		  if (n.name() != node.name()) {
		    auto edge = std::find_if(edges.begin(), edges.end(),
					     [&n, &node](const NavGraphEdge &e)->bool {
					       return
						 (e.from() == node.name() &&
						  e.to() == n.name()) ||
						 (! e.is_directed() &&
						  e.from() == n.name() &&
						  e.to() == node.name());
					     });
		    if (edge == edges.end()) {
		      MenuHandler::EntryHandle undir_handle =
			menu.handler->insert(connect_undir_menu_handle, n.name(),
					     boost::bind(&NavGraphInteractiveThread::process_node_feedback, this, _1));
		      menu.undir_connect_nodes[undir_handle] = n.name();

		      MenuHandler::EntryHandle dir_handle =
			menu.handler->insert(connect_dir_menu_handle, n.name(),
					     boost::bind(&NavGraphInteractiveThread::process_node_feedback, this, _1));
		      menu.dir_connect_nodes[dir_handle] = n.name();
		    } else {
		      MenuHandler::EntryHandle handle =
			menu.handler->insert(disconnect_menu_handle, n.name(),
					     boost::bind(&NavGraphInteractiveThread::process_node_feedback, this, _1));
		      menu.disconnect_nodes[handle] = n.name();
		    }
		  }
		});

  MenuHandler::EntryHandle properties_menu_handle = menu.handler->insert("Properties");
  for (const auto &p : node.properties()) {
    std::string p_s = p.first + ": " + p.second;
    menu.handler->insert(properties_menu_handle, p_s);
  }

  menu.remove_handle =
    menu.handler->insert("Remove Node",
			 boost::bind(&NavGraphInteractiveThread::process_node_feedback,
				     this, _1));

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl pos_control;
  pos_control.orientation_mode =
    visualization_msgs::InteractiveMarkerControl::FIXED;
  pos_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  pos_control.name = "move_x";
  pos_control.orientation.x = 0.;
  pos_control.orientation.y = 0.;
  pos_control.orientation.z = 0.;
  pos_control.orientation.w = 1.;
  int_marker.controls.push_back(pos_control);

  pos_control.name = "move_y";
  pos_control.orientation.x = 0.;
  pos_control.orientation.y = 0.;
  pos_control.orientation.z = 1.;
  pos_control.orientation.w = 1.;
  int_marker.controls.push_back(pos_control);

  if (has_ori) {
    visualization_msgs::InteractiveMarkerControl ori_control;
    ori_control.name = "rot_z";
    ori_control.orientation.w = 1;
    ori_control.orientation.x = 0;
    ori_control.orientation.y = 1;
    ori_control.orientation.z = 0;
    ori_control.interaction_mode =
      visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(ori_control);
    menu.handler->setCheckState(menu.ori_handle, MenuHandler::CHECKED);
  }

  server_->insert(int_marker, boost::bind(&NavGraphInteractiveThread::process_node_feedback, this, _1));

  menu.handler->apply(*server_, int_marker.name);
  node_menus_[int_marker.name] = menu;
}


void
NavGraphInteractiveThread::add_graph(NavGraph *navgraph)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = cfg_global_frame_;
  int_marker.name = navgraph->name();
  int_marker.description = "";
  int_marker.scale = 0.5;

  int_marker.pose.position.x = 0.;
  int_marker.pose.position.y = 0.;
  int_marker.pose.position.z = 1.;
  int_marker.pose.orientation.x = int_marker.pose.orientation.y = int_marker.pose.orientation.z = 0.;
  int_marker.pose.orientation.w = 1.;

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.scale.x = 0.25;
  box_marker.scale.y = 0.25;
  box_marker.scale.z = 0.25;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(box_marker);
  box_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  box_control.description="Graph Ops";
  box_control.name="menu";
  int_marker.controls.push_back(box_control);

  std::shared_ptr<MenuHandler> menu_handler(new MenuHandler());
  graph_menu_.add_handle =
    menu_handler->insert("Add Node",
			 boost::bind(&NavGraphInteractiveThread::process_graph_feedback,
				     this, _1));
  graph_menu_.save_handle =
    menu_handler->insert("Save Graph",
			 boost::bind(&NavGraphInteractiveThread::process_graph_feedback,
				     this, _1));

  graph_menu_.stop_handle =
    menu_handler->insert("STOP",
			 boost::bind(&NavGraphInteractiveThread::process_graph_feedback,
				     this, _1));

  graph_menu_handler_ = menu_handler;

  server_->insert(int_marker,
		  boost::bind(&NavGraphInteractiveThread::process_graph_feedback, this, _1));

  menu_handler->apply(*server_, int_marker.name);
}
