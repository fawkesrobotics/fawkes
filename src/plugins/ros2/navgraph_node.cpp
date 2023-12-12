/***************************************************************************
 *  navgraph_node.cpp
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

#include "navgraph_node.h"

#include "lifecycle_msgs/msg/state.hpp"

#include <interfaces/NavGraphGeneratorInterface.h>
#include <interfaces/NavGraphWithMPSGeneratorInterface.h>
namespace fawkes {

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

ROS2NavgraphNode::ROS2NavgraphNode(const std::string                 &id,
                                   const std::chrono::nanoseconds    &rate,
                                   NavGraphGeneratorInterface        *navgraph_interface,
                                   NavGraphWithMPSGeneratorInterface *navgraph_mps_interface)
: rclcpp_lifecycle::LifecycleNode(id), BlackBoardInterfaceListener("NavgraphNode")
{
	navgraph_interface_     = navgraph_interface;
	navgraph_mps_interface_ = navgraph_mps_interface;
	bbil_add_data_interface(navgraph_interface);
	bbil_add_data_interface(navgraph_mps_interface);
}

ROS2NavgraphNode::~ROS2NavgraphNode()
{
}

CallbackReturn
ROS2NavgraphNode::on_activate(const rclcpp_lifecycle::State &state)
{
	RCLCPP_INFO(rclcpp::get_logger(get_name()), "Starting execution!");

	navgraph_interface_service_ = this->create_service<cx_msgs::srv::NavGraphInterfaceCompute>(
	  "navgraph_compute",
	  [=](const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceCompute::Request> request,
	      std::shared_ptr<cx_msgs::srv::NavGraphInterfaceCompute::Response>      response) {
		  navgraph_compute_received(request, response);
	  });

	navgraph_generate_waitzones_service_ =
	  this->create_service<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones>(
	    "navgraph_compute",
	    [=](const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Request> request,
	        std::shared_ptr<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Response> response) {
		    navgraph_generate_waitzones_received(request, response);
	    });

	navgraph_generate_waitzones_service_ =
	  this->create_service<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones>(
	    "navgraph_compute",
	    [=](const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Request> request,
	        std::shared_ptr<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Response> response) {
		    navgraph_generate_waitzones_received(request, response);
	    });

	navgraph_set_bounding_box_service_ =
	  this->create_service<cx_msgs::srv::NavGraphInterfaceSetBoundingBox>(
	    "navgraph_compute",
	    [=](const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceSetBoundingBox::Request> request,
	        std::shared_ptr<cx_msgs::srv::NavGraphInterfaceSetBoundingBox::Response>      response) {
		    navgraph_set_bounding_box_received(request, response);
	    });

	navgraph_update_station_by_tag_service_ =
	  this->create_service<cx_msgs::srv::NavGraphInterfaceUpdateStationByTag>(
	    "navgraph_compute",
	    [=](const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceUpdateStationByTag::Request> request,
	        std::shared_ptr<cx_msgs::srv::NavGraphInterfaceUpdateStationByTag::Response> response) {
		    navgraph_update_station_by_tag_received(request, response);
	    });

	nav_graph_publisher_ =
	  this->create_publisher<cx_msgs::msg::NavGraphInterfaceMessage>("navgraph-interface", 10);
	nav_graph_with_mps_publisher_ =
	  this->create_publisher<cx_msgs::msg::NavGraphWithMPSInterfaceMessage>(
	    "navgraph-with-mps-interface", 10);

	return rclcpp_lifecycle::LifecycleNode::on_activate(state);
}

void
ROS2NavgraphNode::navgraph_compute_received(
  const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceCompute::Request> request,
  std::shared_ptr<cx_msgs::srv::NavGraphInterfaceCompute::Response>      response)
{
	RCLCPP_INFO(rclcpp::get_logger(get_name()), "Received NavGraphComputeInterface");
	NavGraphGeneratorInterface::ComputeMessage *cm = new NavGraphGeneratorInterface::ComputeMessage();
	navgraph_interface_->msgq_enqueue(cm);
	response->success = true;
}

void
ROS2NavgraphNode::navgraph_generate_waitzones_received(
  const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Request> request,
  std::shared_ptr<cx_msgs::srv::NavGraphInterfaceGenerateWaitzones::Response>      response)
{
	RCLCPP_INFO(rclcpp::get_logger(get_name()), "Received NavGraphGenerateWaitzones");
	NavGraphWithMPSGeneratorInterface::GenerateWaitZonesMessage *gwzm =
	  new NavGraphWithMPSGeneratorInterface::GenerateWaitZonesMessage();
	navgraph_mps_interface_->msgq_enqueue(gwzm);
	response->success = true;
}

void
ROS2NavgraphNode::navgraph_set_bounding_box_received(
  const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceSetBoundingBox::Request> request,
  std::shared_ptr<cx_msgs::srv::NavGraphInterfaceSetBoundingBox::Response>      response)
{
	RCLCPP_INFO(rclcpp::get_logger(get_name()), "Received NavGraphSetBoundingBox");

	NavGraphGeneratorInterface::SetBoundingBoxMessage *sbbm =
	  new NavGraphGeneratorInterface::SetBoundingBoxMessage(request->p1_x,
	                                                        request->p1_y,
	                                                        request->p2_x,
	                                                        request->p2_y);
	navgraph_interface_->msgq_enqueue(sbbm);
	response->success = true;
}

void
ROS2NavgraphNode::navgraph_update_station_by_tag_received(
  const std::shared_ptr<cx_msgs::srv::NavGraphInterfaceUpdateStationByTag::Request> request,
  std::shared_ptr<cx_msgs::srv::NavGraphInterfaceUpdateStationByTag::Response>      response)
{
	RCLCPP_INFO(rclcpp::get_logger(get_name()), "Received NavGraphUpdateStationByTag");

	NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage *usbtm =
	  new NavGraphWithMPSGeneratorInterface::UpdateStationByTagMessage();

	usbtm->set_name(request->name.c_str());
	if (request->side == "INPUT")
		usbtm->set_side(NavGraphWithMPSGeneratorInterface::Side::INPUT);
	else if (request->side == "OUTPUT")
		usbtm->set_side(NavGraphWithMPSGeneratorInterface::Side::OUTPUT);
	usbtm->set_frame(request->frame.c_str());
	usbtm->set_tag_translation(0, request->tag_translation[0]);
	usbtm->set_tag_translation(1, request->tag_translation[1]);
	usbtm->set_tag_translation(2, request->tag_translation[2]);
	usbtm->set_tag_rotation(0, request->tag_rotation[0]);
	usbtm->set_tag_rotation(1, request->tag_rotation[1]);
	usbtm->set_tag_rotation(2, request->tag_rotation[2]);
	usbtm->set_tag_rotation(3, request->tag_rotation[3]);
	usbtm->set_zone_coords(0, request->zone_coords[0]);
	usbtm->set_zone_coords(1, request->zone_coords[1]);

	navgraph_mps_interface_->msgq_enqueue(usbtm);
	response->success = true;
}

/** Handle interface changes.
 * If either of the navgraph interfaces changes, publish updated data to ROS.
 * @param interface interface instance that you supplied to bbil_add_data_interface()
 */
void
ROS2NavgraphNode::bb_interface_data_refreshed(Interface *interface) noexcept
{
	RCLCPP_INFO(rclcpp::get_logger(get_name()), "GOT MESSAGE FROM INTERFACE '%s'", interface->type());
	if (interface->type() == "NavGraphWithMPSGeneratorInterface") {
		NavGraphWithMPSGeneratorInterface *navgraph_mps_interface =
		  dynamic_cast<NavGraphWithMPSGeneratorInterface *>(interface);
		if (!navgraph_mps_interface)
			return;
		navgraph_mps_interface->read();

		bool final = navgraph_mps_interface->is_final();

		auto message  = cx_msgs::msg::NavGraphWithMPSInterfaceMessage();
		message.final = final;

		nav_graph_with_mps_publisher_->publish(message);

	} else if (interface->type() == "NavGraphGeneratorInterface") {
		NavGraphGeneratorInterface *navgraph_interface =
		  dynamic_cast<NavGraphGeneratorInterface *>(interface);
		if (!navgraph_interface)
			return;
		navgraph_interface->read();

		bool        final     = navgraph_interface->is_final();
		bool        ok        = navgraph_interface->is_ok();
		std::string error_msg = navgraph_interface->error_message();

		auto message      = cx_msgs::msg::NavGraphInterfaceMessage();
		message.final     = final;
		message.ok        = ok;
		message.error_msg = error_msg;

		nav_graph_publisher_->publish(message);
	}
}
} // namespace fawkes
