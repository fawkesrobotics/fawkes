
/***************************************************************************
 *  skiller-rest-api.cpp -  CLIPS Executive REST API
 *
 *  Created: Sun Mar 25 01:29:29 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
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

#include "skiller-rest-api.h"

#include <webview/rest_api_manager.h>
#include <core/threading/mutex_locker.h>
#include <utils/time/wait.h>

#include <interfaces/SkillerDebugInterface.h>
#include <interfaces/SkillerInterface.h>

using namespace fawkes;

/** @class SkillerRestApi "skiller-rest-api.h"
 * REST API backend for the CLIPS executive.
 * @author Tim Niemueller
 */

/** Constructor. */
SkillerRestApi::SkillerRestApi()
	: Thread("ClipsWebviewThread", Thread::OPMODE_WAITFORWAKEUP)
{
}

/** Destructor. */
SkillerRestApi::~SkillerRestApi()
{
}

void
SkillerRestApi::init()
{
	skdb_if_    = blackboard->open_for_reading<SkillerDebugInterface>("Skiller");
	skiller_if_ = blackboard->open_for_reading<SkillerInterface>("Skiller");

	rest_api_ = new WebviewRestApi("skiller", logger);
	rest_api_->add_handler<WebviewRestArray<SkillInfo>>
		(WebRequest::METHOD_GET, "/skills",
		 std::bind(&SkillerRestApi::cb_list_skills, this));
	rest_api_->add_handler<Skill>
		(WebRequest::METHOD_GET, "/skills/{id}",
		 std::bind(&SkillerRestApi::cb_get_skill, this, std::placeholders::_1));
	rest_api_->add_handler
		(WebRequest::METHOD_DELETE, "/skills/{id}",
		 std::bind(&SkillerRestApi::cb_stop_skill, this, std::placeholders::_1));
	rest_api_->add_handler<Skill, SkillCall>
		(WebRequest::METHOD_POST, "/call",
		 std::bind(&SkillerRestApi::cb_exec_skill, this, std::placeholders::_1));
	webview_rest_api_manager->register_api(rest_api_);
}

void
SkillerRestApi::finalize()
{
	webview_rest_api_manager->unregister_api(rest_api_);
	delete rest_api_;
}


void
SkillerRestApi::loop()
{
}

void
SkillerRestApi::set_and_wait_graph(const char *graph)
{
	if (strcmp(skdb_if_->graph_fsm(), graph) != 0) {
		// It's not currently the desired graph
		SkillerDebugInterface::SetGraphMessage *m =
			new SkillerDebugInterface::SetGraphMessage(graph);
		skdb_if_->msgq_enqueue(m);
		fawkes::Time start(clock);
		fawkes::Time now(clock);
		do {
			fawkes::TimeWait::wait_systime(50000);
			skdb_if_->read();
			now.stamp();
		} while (strcmp(skdb_if_->graph_fsm(), graph) != 0 &&
		         (now - &start) <= 5.0);
	}

	if (strcmp(skdb_if_->graph_fsm(), graph) != 0) {
		throw WebviewRestException(WebReply::HTTP_REQUEST_TIMEOUT,
		                           "Did not receive '%s' in time from skiller", graph);
	}
}

WebviewRestArray<SkillInfo>
SkillerRestApi::cb_list_skills()
{
	WebviewRestArray<SkillInfo> rv;

	skdb_if_->read();
	if (! skdb_if_->has_writer() || ! skiller_if_->has_writer()) {
		throw WebviewRestException(WebReply::HTTP_SERVICE_UNAVAILABLE,
		                           "Behavior Engine plugin is not loaded");
	}

	std::string prev_fsm = skdb_if_->graph_fsm();

	set_and_wait_graph("LIST");

	std::stringstream ss(skdb_if_->graph());
	std::string skill_name;
	while (std::getline(ss, skill_name, '\n')) {
		SkillInfo s;
		s.set_kind("SkillInfo");
		s.set_apiVersion(SkillInfo::api_version());
		s.set_name(skill_name);
		rv.push_back(std::move(s));
	}

	SkillerDebugInterface::SetGraphMessage *m =
		new SkillerDebugInterface::SetGraphMessage(prev_fsm.c_str());
	skdb_if_->msgq_enqueue(m);

	return rv;
}

Skill
SkillerRestApi::cb_get_skill(WebviewRestParams& params)
{
	std::string skill_name{params.path_arg("id")};

	if (skill_name == "active") {
		skill_name = "ACTIVE";
	}

	skdb_if_->read();
	skiller_if_->read();
	if (! skdb_if_->has_writer() || ! skiller_if_->has_writer()) {
		throw WebviewRestException(WebReply::HTTP_SERVICE_UNAVAILABLE,
		                           "Behavior Engine plugin is not loaded");
	}

	set_and_wait_graph(skill_name.c_str());

	Skill s;
	s.set_kind("Skill");
	s.set_apiVersion(Skill::api_version());
	s.set_name(skdb_if_->graph_fsm());
	s.set_graph(skdb_if_->graph());

	if (skill_name == "ACTIVE") {
		skiller_if_->read();
		s.set_skill_string(skiller_if_->skill_string());
		s.set_error(skiller_if_->error());
		s.set_msg_id(skiller_if_->msgid());
		s.set_exclusive_controller(skiller_if_->exclusive_controller());
		switch (skiller_if_->status()) {
		case SkillerInterface::S_RUNNING:
			s.set_status("RUNNING");
			break;
		case SkillerInterface::S_FINAL:
			s.set_status("FINAL");
			break;
		case SkillerInterface::S_FAILED:
			s.set_status("FAILED");
			break;
		default:
			s.set_status("INACTIVE");
			break;
		}	
	}

	return s;
}


Skill
SkillerRestApi::cb_exec_skill(const SkillCall &call)
{
	if (! call.skill_string()) {
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
		                           "Request lacks skill string");
	}

	skiller_if_->read();
	if (! skiller_if_->has_writer()) {
		throw WebviewRestException(WebReply::HTTP_SERVICE_UNAVAILABLE,
		                           "Behavior Engine plugin is not loaded");
	}

	if (skiller_if_->exclusive_controller() != 0) {
		throw WebviewRestException(WebReply::HTTP_CONFLICT,
		                           "Another thread is exclusive controller");
	}

	SkillerInterface::ExecSkillMessage *m =
		new SkillerInterface::ExecSkillMessage(call.skill_string()->c_str());
	m->ref();

	try {
		skiller_if_->msgq_enqueue(m);
	} catch (Exception &e) {
		logger->log_error(name(), "Failed to execute skill: %s", e.what_no_backtrace());
		throw WebviewRestException(WebReply::HTTP_INTERNAL_SERVER_ERROR,
		                           "Failed to execute skill: %s", e.what_no_backtrace());
	}

	Skill sk;
	sk.set_kind("Skill");
	sk.set_apiVersion(Skill::api_version());
	sk.set_name("active");
	sk.set_msg_id(m->id());
	m->unref();
	return sk;
}


std::unique_ptr<fawkes::WebviewRestReply>
SkillerRestApi::cb_stop_skill(WebviewRestParams& params)
{
	std::string skill_name{params.path_arg("id")};

	if (skill_name != "active") {
		throw WebviewRestException(WebReply::HTTP_BAD_REQUEST,
		                           "Only the 'active' skill can be stopped");
	}

	skiller_if_->read();
	if (! skiller_if_->has_writer()) {
		throw WebviewRestException(WebReply::HTTP_SERVICE_UNAVAILABLE,
		                           "Behavior Engine plugin is not loaded");
	}

	if (skiller_if_->exclusive_controller() != 0) {
		throw WebviewRestException(WebReply::HTTP_CONFLICT,
		                           "Another thread is exclusive controller");
	}

	try {
		skiller_if_->msgq_enqueue(new SkillerInterface::StopExecMessage());
	} catch (Exception &e) {
		logger->log_error(name(), "Failed to stop skill: %s", e.what_no_backtrace());
		throw WebviewRestException(WebReply::HTTP_INTERNAL_SERVER_ERROR,
		                           "Failed to execute skill: %s", e.what_no_backtrace());
	}

	return std::make_unique<WebviewRestReply>(WebReply::HTTP_OK, "OK", "text/plain");
}
