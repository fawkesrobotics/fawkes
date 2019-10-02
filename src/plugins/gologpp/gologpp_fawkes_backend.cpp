/***************************************************************************
 *  gologpp_fawkes_backend.cpp - Fawkes backend for Golog++
 *
 *  Created: Mon 26 Aug 2019 CEST 15:38
 *  Copyright  2019  Victor Mataré <matare@fh-aachen.de>
 *                   Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include "gologpp_fawkes_backend.h"

#include <golog++/model/activity.h>
#include <golog++/model/transition.h>
#include <interfaces/SkillerInterface.h>

#include <sstream>

namespace fawkes_gpp {

using namespace gologpp;
using namespace fawkes;

/** @class GologppFawkesBackend
 *  A Golog++ backend to get data from and send commands to Fawkes.
 *  The backend currently only provides access to the skiller for action
 *  execution.
 */

/** Constructor.
 *  @param main_thread the main thread of the Golog++ plugin
 *  @param logger The logger to use for log messages
 *  @param blackboard The blackboard to use to access the skiller
 */
GologppFawkesBackend::GologppFawkesBackend(GologppThread *main_thread,
                                           Logger *       logger,
                                           BlackBoard *   blackboard)
: BlackBoardInterfaceListener("gologpp_agent"),
  main_thread_(main_thread),
  logger_(logger),
  blackboard_(blackboard)
{
	try {
		skiller_if_ = blackboard_->open_for_reading<SkillerInterface>("Skiller");
	} catch (Exception &e) {
		logger_->log_error(name(), "Failed to open skiller interface: %s", e.what_no_backtrace());
	}
	bbil_add_data_interface(skiller_if_);
	blackboard_->register_listener(this, BlackBoard::BBIL_FLAG_DATA);
	skiller_if_->read();
	if (!skiller_if_->has_writer()) {
		blackboard->unregister_listener(this);
		blackboard->close(skiller_if_);
		throw Exception("No writer for Skiller interface");
	}
	skiller_if_->msgq_enqueue(new SkillerInterface::AcquireControlMessage());
}

GologppFawkesBackend::~GologppFawkesBackend()
{
	skiller_if_->msgq_enqueue(new SkillerInterface::ReleaseControlMessage());
	blackboard_->unregister_listener(this);
	blackboard_->close(skiller_if_);
}

/** Preempt the currently running activity.
 *  If the currently running activity is the given activity, stop it, otherwise do nothing.
 *  @param t The transition to do, used to check if this is the currently running activity.
 */
void
GologppFawkesBackend::preempt_activity(shared_ptr<Transition> t)
{
	if (*running_activity_ == *t) {
		stop_running_activity();
	}
}

/** Get the current time from Fawkes.
 *  @return the current time
 */
gologpp::Clock::time_point
GologppFawkesBackend::time() const noexcept
{
	return gologpp::Clock::time_point{
	  gologpp::Clock::duration{clock->now().in_sec() / gologpp::Clock::duration::period::den}};
}

/** Execute the given activity using the skiller.
 *  Construct a skill string from the activity and send it to the skiller.
 *  @param a The activity to start.
 */
void
GologppFawkesBackend::execute_activity(shared_ptr<Activity> a)
{
	if (running_activity_) {
		stop_running_activity();
	}
	std::stringstream skill_string;
	skill_string << a->mapped_name() << "{";
	std::string arg_separator{""};
	for (auto &arg : a->target()->mapping().arg_mapping()) {
		skill_string << arg_separator;
		auto        v         = a->mapped_arg_value(arg.first);
		std::string arg_quote = "";
		if (v.type().is<gologpp::StringType>()) {
			arg_quote = "\"";
		}
		skill_string << arg.first << "=" << arg_quote << a->mapped_arg_value(arg.first) << arg_quote;
		arg_separator = ", ";
	}
	skill_string << "}";

	skiller_if_->msgq_enqueue(new SkillerInterface::ExecSkillMessage(skill_string.str().c_str()));
	running_activity_ = a;
}

void
GologppFawkesBackend::stop_running_activity()
{
	skiller_if_->msgq_enqueue(new SkillerInterface::StopExecMessage());
	running_activity_.reset();
}

void
GologppFawkesBackend::bb_interface_data_changed(Interface *iface) throw()
{
	if (!running_activity_) {
		return;
	}
	SkillerInterface *skiller_if = dynamic_cast<SkillerInterface *>(iface);
	if (!skiller_if) {
		return;
	}
	skiller_if->read();
	switch (skiller_if->status()) {
	case SkillerInterface::S_FINAL: running_activity_->update(Transition::Hook::FINISH); break;
	case SkillerInterface::S_FAILED: running_activity_->update(Transition::Hook::FAIL); break;
	case SkillerInterface::S_RUNNING: running_activity_->update(Transition::Hook::START); break;
	default: break;
	}
}

const char *
GologppFawkesBackend::name()
{
	return main_thread_->name();
}

} // namespace fawkes_gpp
