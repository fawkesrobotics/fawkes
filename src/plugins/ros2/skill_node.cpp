/***************************************************************************
 *  skill_node.cpp
 *
 *  Created: 04 August 2023
 *  Copyright  2023 Daniel Swoboda
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
#include "skill_node.h"

#include "lifecycle_msgs/msg/state.hpp"

#include <interfaces/SkillerInterface.h>
namespace fawkes {

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

SkillNode::SkillNode(const std::string              &id,
                     const std::string              &robot,
                     const std::string              &executor,
                     const std::chrono::nanoseconds &rate,
                     fawkes::SkillerInterface       *skiller_if)
: SkillExecution(id, rate), BlackBoardInterfaceListener("SkillNode"), skiller_if_(skiller_if)
{
	robot_id_    = robot;
	executor_id_ = executor;
	bbil_add_data_interface(skiller_if);
	SkillerInterface::AcquireControlMessage *aqm = new SkillerInterface::AcquireControlMessage();
	skiller_if->msgq_enqueue(aqm);
}

SkillNode::~SkillNode()
{
	SkillerInterface::ReleaseControlMessage *rcm = new SkillerInterface::ReleaseControlMessage();
	skiller_if_->msgq_enqueue(rcm);
}

CallbackReturn
SkillNode::on_activate(const rclcpp_lifecycle::State &state)
{
	sent_command_ = false;
	SkillerInterface::ExecSkillMessage *esm =
	  new SkillerInterface::ExecSkillMessage(mapped_action_.c_str());

	skiller_if_->msgq_enqueue(esm);
	RCLCPP_INFO_STREAM(get_logger(), "Starting execution!");

	return SkillExecution::on_activate(state);
}

void
SkillNode::perform_execution()
{
	RCLCPP_INFO_STREAM(get_logger(), "Polling");
}

/** Handle interface changes.
 * If the Skiller interface changes, publish updated data to ROS.
 * @param interface interface instance that you supplied to bbil_add_data_interface()
 */
void
SkillNode::bb_interface_data_refreshed(Interface *interface) noexcept
{
	SkillerInterface *skiller_if = dynamic_cast<SkillerInterface *>(interface);
	if (!skiller_if)
		return;
	skiller_if->read();
	if (skiller_if->serial().get_string() != std::string(skiller_if->exclusive_controller())) {
		finish_execution(false, 0.0, "Skiller Control Lost");
		return;
	}
	if (!sent_command_) {
		switch (skiller_if->status()) {
		case fawkes::SkillerInterface::SkillStatusEnum::S_INACTIVE: send_feedback(0.0, "inactive");
		case fawkes::SkillerInterface::SkillStatusEnum::S_FINAL:
			finish_execution(true, 1.0, "completed");
		case fawkes::SkillerInterface::SkillStatusEnum::S_RUNNING: send_feedback(0.0, "running");
		case fawkes::SkillerInterface::SkillStatusEnum::S_FAILED:
			finish_execution(true, 1.0, skiller_if->error());
		}
	}
}

} // namespace fawkes
