/***************************************************************************
 *  skill_node.hp
 *
 *  Created: 04 August 2023
 *  Copyright  2023 Tarik Viehmann
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

#ifndef SKILL_NODE_H
#define SKILL_NODE_H

#include <aspect/blackboard.h>
#include <blackboard/interface_listener.h>
#include <interfaces/SkillerInterface.h>

#include <chrono>
#include <cx_msgs/msg/skill_action_exec_info.hpp>
#include <cx_msgs/msg/skill_execution.hpp>
#include <cx_skill_execution/SkillExecution.hpp>
#include <fstream>
#include <iostream>
#include <lifecycle_msgs/msg/state.hpp>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <regex>
#include <string>

namespace fawkes {
using namespace std::chrono_literals;

class SkillNode : public cx::SkillExecution, public fawkes::BlackBoardInterfaceListener
{
	using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
	SkillNode(const std::string              &id,
	          const std::string              &robot,
	          const std::string              &executor,
	          const std::chrono::nanoseconds &rate,
	          fawkes::SkillerInterface       *skiller_if);
	~SkillNode();

	CallbackReturn on_activate(const rclcpp_lifecycle::State &state);

	void         perform_execution() override;
	virtual void bb_interface_data_refreshed(fawkes::Interface *interface) noexcept override;

private:
	fawkes::SkillerInterface *skiller_if_;

	bool sent_command_ = false;
	bool executing_    = false;
};
} // namespace fawkes
#endif // !SKILL_NAODE_H
