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

#ifndef __PLUGINS_NAVGRAPH_INTERACTIVE_NAVGRAPH_INTERACTIVE_THREAD_H_
#define __PLUGINS_NAVGRAPH_INTERACTIVE_NAVGRAPH_INTERACTIVE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/aspect_provider.h>
#include <plugins/ros/aspect/ros.h>
#include <navgraph/aspect/navgraph.h>

#include <interfaces/NavigatorInterface.h>

#include <navgraph/navgraph.h>
#include <navgraph/navgraph_node.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <interactive_markers/menu_handler.h>

#include <memory>

namespace interactive_markers {
  class InteractiveMarkerServer;
}

class NavGraphInteractiveThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ROSAspect,
  public fawkes::NavGraphAspect
{
 private:
  typedef struct {
    std::shared_ptr<interactive_markers::MenuHandler> handler;
    interactive_markers::MenuHandler::EntryHandle  ori_handle;
    interactive_markers::MenuHandler::EntryHandle  goto_handle;
    interactive_markers::MenuHandler::EntryHandle  remove_handle;
    std::map<interactive_markers::MenuHandler::EntryHandle, std::string> dir_connect_nodes;
    std::map<interactive_markers::MenuHandler::EntryHandle, std::string> undir_connect_nodes;
    std::map<interactive_markers::MenuHandler::EntryHandle, std::string> disconnect_nodes;
  } NodeMenu;

  typedef struct {
    interactive_markers::MenuHandler::EntryHandle  add_handle;
    interactive_markers::MenuHandler::EntryHandle  save_handle;
    interactive_markers::MenuHandler::EntryHandle  stop_handle;
  } GraphMenu;

 public:
  NavGraphInteractiveThread();
  virtual ~NavGraphInteractiveThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run();}

 private:
  void process_node_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void process_node_ori_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
				 const NodeMenu &menu,
				 visualization_msgs::InteractiveMarker &int_marker);
  void process_graph_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void add_graph(fawkes::NavGraph *navgraph);
  void add_node(const fawkes::NavGraphNode &node, fawkes::NavGraph *navgraph);

 private:
  std::string cfg_global_frame_;
  std::string cfg_save_filename_;

  interactive_markers::InteractiveMarkerServer *server_;

  std::map<std::string, NodeMenu> node_menus_;

  std::shared_ptr<interactive_markers::MenuHandler> graph_menu_handler_;
  GraphMenu graph_menu_;

  fawkes::NavigatorInterface *nav_if_;
};

#endif
