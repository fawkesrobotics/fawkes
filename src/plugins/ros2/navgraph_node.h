/***************************************************************************
 *  navgraph_node.h
 *
 *  Created: Dec 2023
 *  Copyright  2023 Daniel Swoboda <swoboda@kbsg.rwth-aachen.de>
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

#ifndef NAVGRAPH_NODE_H
#define NAVGRAPH_NODE_H

#include <aspect/blackboard.h>
#include <blackboard/interface_listener.h>
#include <interfaces/NavGraphGeneratorInterface.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>

#include <chrono>
#include <cx_msgs/msg/nav_graph_interface_message.hpp>
#include <cx_msgs/msg/nav_graph_with_mps_interface_message.hpp>
#include <cx_msgs/srv/nav_graph_interface_compute.hpp>
#include <cx_msgs/srv/nav_graph_interface_generate_waitzones.hpp>
#include <cx_msgs/srv/nav_graph_interface_set_bounding_box.hpp>
#include <cx_msgs/srv/nav_graph_interface_update_station_by_tag.hpp>
#include <fstream>
#include <iostream>
#include <lifecycle_msgs/msg/state.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <regex>
#include <string>

namespace fawkes {
using namespace std::chrono_literals;

class ROS2NavgraphNode : public rclcpp_lifecycle::LifecycleNode,
                         public fawkes::BlackBoardInterfaceListener
{
	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
	ROS2NavgraphNode(const std::string                         &id,
	                 const std::chrono::nanoseconds            &rate,
	                 fawkes::NavGraphGeneratorInterface        *navgraph_interface,
	                 fawkes::NavGraphWithMPSGeneratorInterface *navgraph_mps_interface);
	~ROS2NavgraphNode();

	CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

	void navgraph_compute_received(
	  const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceCompute::Request> request,
	  std::shared_ptr<cx_msgs::srv::NavGraphInterfaceCompute::Response>      response);

	void navgraph_generate_waitzones_received(
	  const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Request> request,
	  std::shared_ptr<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Response>      response);

	void navgraph_set_bounding_box_received(
	  const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceSetBoundingBox::Request> request,
	  std::shared_ptr<cx_msgs::srv::NavGraphInterfaceSetBoundingBox::Response>      response);

	void navgraph_update_station_by_tag_received(
	  const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceUpdateStationByTag::Request> request,
	  std::shared_ptr<cx_msgs::srv::NavGraphInterfaceUpdateStationByTag::Response>      response);

	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) noexcept override;

private:
	fawkes::NavGraphGeneratorInterface                                *navgraph_interface_;
	fawkes::NavGraphWithMPSGeneratorInterface                         *navgraph_mps_interface_;
	rclcpp::Service<cx_msgs::srv::NavGraphInterfaceCompute>::SharedPtr navgraph_interface_service_;
	rclcpp::Service<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones>::SharedPtr
	  navgraph_generate_waitzones_service_;
	rclcpp::Service<cx_msgs::srv::NavGraphInterfaceSetBoundingBox>::SharedPtr
	  navgraph_set_bounding_box_service_;
	rclcpp::Service<cx_msgs::srv::NavGraphInterfaceUpdateStationByTag>::SharedPtr
	  navgraph_update_station_by_tag_service_;
	rclcpp::Publisher<cx_msgs::msg::NavGraphInterfaceMessage>::SharedPtr nav_graph_publisher_;
	rclcpp::Publisher<cx_msgs::msg::NavGraphWithMPSInterfaceMessage>::SharedPtr
	  nav_graph_with_mps_publisher_;
};
} // namespace fawkes
#endif // !NAVGRAPH_NODE_H