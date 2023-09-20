/***************************************************************************
 *  exec_thread.cpp - Simulate skill execution
 *
 *  Created: Mon 06 May 2019 08:51:53 CEST 08:51
 *  Copyright  2019  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "exec_thread.h"

using namespace std;
using namespace fawkes;

/** @class SkillerSimulatorExecutionThread "exec_thread.h"
 *  Simulated Skill Execution Thread.
 *  This thread pretends to execute a skill, similar to the real skiller execution thread.
 *
 *  @see SkillerExecutionThread
 *  @author Till Hofmann
 */

/** Constructor. */
SkillerSimulatorExecutionThread::SkillerSimulatorExecutionThread()
: Thread("SkillerSimulatorExecutionThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SKILL)
{
}

void
SkillerSimulatorExecutionThread::init()
{
	skiller_if_      = blackboard->open_for_writing<SkillerInterface>("Skiller");
	skill_starttime_ = Time();
}

void
SkillerSimulatorExecutionThread::loop()
{
	bool skill_enqueued  = false;
	bool write_interface = false;
	while (!skiller_if_->msgq_empty()) {
		if (skiller_if_->msgq_first_is<SkillerInterface::AcquireControlMessage>()) {
			SkillerInterface::AcquireControlMessage *m =
			  skiller_if_->msgq_first<SkillerInterface::AcquireControlMessage>();
			if (strcmp(skiller_if_->exclusive_controller(), "") == 0) {
				const std::string new_controller = m->source_id().get_string();
				logger->log_debug(name(),
				                  "%s is new exclusive controller (ID %s)",
				                  m->sender_thread_name(),
				                  new_controller.c_str());
				skiller_if_->set_exclusive_controller(new_controller.c_str());
				skiller_if_->set_msgid(m->id());
				write_interface = true;
			} else if (m->is_steal_control()) {
				const std::string new_controller = m->source_id().get_string();
				logger->log_warn(name(),
				                 "%s steals exclusive control (ID %s)",
				                 m->sender_thread_name(),
				                 new_controller.c_str());
				skiller_if_->set_exclusive_controller(new_controller.c_str());
				skiller_if_->set_msgid(m->id());
				write_interface = true;
			} else {
				logger->log_warn(
				  name(),
				  "%s tried to acquire exclusive control, but another controller exists already",
				  m->sender_thread_name());
			}
		} else if (skiller_if_->msgq_first_is<SkillerInterface::ReleaseControlMessage>()) {
			SkillerInterface::ReleaseControlMessage *m =
			  skiller_if_->msgq_first<SkillerInterface::ReleaseControlMessage>();
			if (skiller_if_->exclusive_controller() == m->source_id().get_string()) {
				logger->log_debug(name(), "%s releases exclusive control", m->sender_thread_name());
			} else if (strcmp(skiller_if_->exclusive_controller(), "") != 0) {
				logger->log_warn(name(),
				                 "%s tried to release exclusive control, but it's not the controller",
				                 m->sender_thread_name());
			}
		} else if (skiller_if_->msgq_first_is<SkillerInterface::ExecSkillMessage>()) {
			SkillerInterface::ExecSkillMessage *m =
			  skiller_if_->msgq_first<SkillerInterface::ExecSkillMessage>();
			if (strcmp(skiller_if_->exclusive_controller(), "") == 0
			    || skiller_if_->exclusive_controller() == m->source_id().get_string()) {
				if (skill_enqueued) {
					logger->log_warn(name(),
					                 "More than one skill string enqueued, ignoring previous string (%s).",
					                 skiller_if_->skill_string());
				}
				if (strcmp(skiller_if_->exclusive_controller(), "") == 0) {
					std::string sender = m->sender_thread_name();
					if (sender == "Unknown") {
						sender = "Remote";
					}
					logger->log_info(name(),
					                 "%s executed '%s' without any exclusive controller",
					                 sender.c_str(),
					                 m->skill_string());
				} else {
					logger->log_info(name(), "%s executes '%s'", m->sender_thread_name(), m->skill_string());
				}
			}
			if (skiller_if_->status() == SkillerInterface::S_RUNNING) {
				logger->log_info(name(),
				                 "Aborting execution of previous skill string '%s' for new goal",
				                 skiller_if_->skill_string());
			}
			skiller_if_->set_skill_string(m->skill_string());
			skiller_if_->set_msgid(m->id());
			skiller_if_->set_error("");
			skiller_if_->set_status(SkillerInterface::S_RUNNING);
			current_skill_runtime_ = get_skill_runtime(m->skill_string());
			start_execute_skill(m->skill_string());
			logger->log_info(name(),
			                 "Executing '%s', will take %.2f seconds",
			                 m->skill_string(),
			                 current_skill_runtime_);
			skill_starttime_ = Time();
			write_interface  = true;
			skill_enqueued   = true;
		} else if (skiller_if_->msgq_first_is<SkillerInterface::StopExecMessage>()) {
			SkillerInterface::StopExecMessage *m =
			  skiller_if_->msgq_first<SkillerInterface::StopExecMessage>();
			if (skiller_if_->exclusive_controller() == m->source_id().get_string()) {
				logger->log_debug(name(),
				                  "Stopping execution of '%s' on request",
				                  skiller_if_->skill_string());
				skiller_if_->set_skill_string("");
				skiller_if_->set_error("");
				skiller_if_->set_msgid(m->id());
				skiller_if_->set_status(SkillerInterface::S_INACTIVE);
				write_interface = true;
			} else {
				std::string sender = m->sender_thread_name();
				if (sender == "Unknown") {
					sender = "Remote";
				}
				logger->log_debug(name(), "%s sent stop without any exclusive controller", sender.c_str());
			}
		} else {
			logger->log_warn(name(), "Unhandled message in skiller interface");
		}
		skiller_if_->msgq_pop();
	}

	if (!skill_enqueued) {
		if (skiller_if_->status() == SkillerInterface::S_RUNNING) {
			Time now = Time();
			if (Time() > skill_starttime_ + current_skill_runtime_) {
				logger->log_info(name(), "Skill '%s' is final", skiller_if_->skill_string());
				auto [exec_status, error] = end_execute_skill(skiller_if_->skill_string());
				skiller_if_->set_skill_string(skiller_if_->skill_string());
				skiller_if_->set_error(error.c_str());
				skiller_if_->set_status(exec_status);
				write_interface = true;
			}
		}
	}

	if (write_interface) {
		skiller_if_->write();
	}
}

void
SkillerSimulatorExecutionThread::finalize()
{
	blackboard->close(skiller_if_);
}

float
SkillerSimulatorExecutionThread::get_skill_runtime(const std::string &skill) const
{
	auto provider = execution_time_estimator_manager_->get_provider(skill);
	return provider->get_execution_time(skill);
}

void
SkillerSimulatorExecutionThread::start_execute_skill(const std::string &skill)
{
	auto provider = execution_time_estimator_manager_->get_provider(skill);
	return provider->start_execute(skill);
}

std::pair<fawkes::SkillerInterface::SkillStatusEnum, std::string>
SkillerSimulatorExecutionThread::end_execute_skill(const std::string &skill)
{
	auto provider = execution_time_estimator_manager_->get_provider(skill);
	return provider->end_execute(skill);
}
