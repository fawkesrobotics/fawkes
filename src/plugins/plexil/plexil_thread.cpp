
/***************************************************************************
 *  plexil_thread.cpp -  PLEXIL executive
 *
 *  Created: Mon Aug 13 11:20:12 2018
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

#include "plexil_thread.h"
#include "log_stream.h"
#include "clock_adapter.h"
#include "log_adapter.h"
#include "be_adapter.h"

#include <core/threading/mutex_locker.h>

#include <ExecApplication.hh>
#include <Debug.hh>
#include <InterfaceSchema.hh>
#include <InterfaceManager.hh>
#include <pugixml.hpp>

#include <fstream>
#include <cstring>

using namespace fawkes;

/** @class PlexilExecutiveThread "plexil_thread.h"
 * Main thread of PLEXIL executive.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
PlexilExecutiveThread::PlexilExecutiveThread()
: Thread("PlexilExecutiveThread", Thread::OPMODE_CONTINUOUS)
{
	set_prepfin_conc_loop(true);
}


/** Destructor. */
PlexilExecutiveThread::~PlexilExecutiveThread()
{
}


static void replace_tokens(std::string &s) {
	std::map<std::string, std::string> tokens =
	  {{"@CFGDIR@", CONFDIR},
	   {"@BASEDIR@", BASEDIR},
	   {"@FAWKES_BASEDIR@", FAWKES_BASEDIR},
	   {"@RESDIR@", RESDIR}};

	for (const auto &token : tokens) {
		std::string::size_type pos;
		if ((pos = s.find(token.first)) != std::string::npos) {
			s.replace(pos, token.first.size(), token.second);
		}
	}
}

void
PlexilExecutiveThread::init()
{
	cfg_spec_ = config->get_string("/plexil/spec");

	std::string cfg_prefix = "/plexil/" + cfg_spec_ + "/";
	std::vector<std::string> cfg_adapters =
	  config->get_strings_or_defaults((cfg_prefix + "adapters").c_str(), {});

	bool cfg_print_xml =
	  config->get_bool_or_default((cfg_prefix + "debug/print-xml").c_str(), false);

	plexil_.reset(new PLEXIL::ExecApplication);

	PLEXIL::g_manager->setProperty("::Fawkes::Clock", clock);
	PLEXIL::g_manager->setProperty("::Fawkes::Logger", logger);
	PLEXIL::g_manager->setProperty("::Fawkes::BlackBoard", blackboard);

	clock_adapter_ = new PLEXIL::ConcreteAdapterFactory<ClockPlexilTimeAdapter>("FawkesTime");
	log_adapter_   = new PLEXIL::ConcreteAdapterFactory<LoggingPlexilAdapter>("FawkesLogging");
	be_adapter_    = new PLEXIL::ConcreteAdapterFactory<BehaviorEnginePlexilAdapter>("BehaviorEngine");

	pugi::xml_document xml_config;
	pugi::xml_node xml_interfaces =
	  xml_config.append_child(PLEXIL::InterfaceSchema::INTERFACES_TAG());

	for (const auto &a : cfg_adapters) {
		if (a == "Utility") {
			logger->log_warn(name(), "Utility adapter configured, consider using FawkesLogging instead");
		} else if (a == "OSNativeTime") {
			logger->log_warn(name(), "OSNativeTime adapter configured, consider using FawkesTime instead");
		}
		pugi::xml_node xml_adapter = xml_interfaces.append_child(PLEXIL::InterfaceSchema::ADAPTER_TAG());
		xml_adapter.append_attribute("AdapterType").set_value(a.c_str());
	}

	if (cfg_print_xml) {
		struct xml_string_writer: pugi::xml_writer
		{
			std::string result;
			virtual void write(const void* data, size_t size)
			{
				result.append(static_cast<const char*>(data), size);
			}
		};

		xml_string_writer writer;
		xml_config.save(writer);
		logger->log_info(name(), "Interface config XML:\n%s", writer.result.c_str());
	}

	std::string cfg_debug_conf = config->get_string_or_default((cfg_prefix + "debug/conf").c_str(), "");
	if (! cfg_debug_conf.empty()) {
		replace_tokens(cfg_debug_conf);

		std::ifstream dbg_f(cfg_debug_conf);
		if (dbg_f.good()) {
			PLEXIL::readDebugConfigStream(dbg_f);
		} else {
			logger->log_warn(name(), "Error opening debug config: %s", strerror(errno));
		}
	}

	log_buffer_.reset(new PlexilLogStreamBuffer(logger));
	log_stream_.reset(new std::ostream(&*log_buffer_));
	PLEXIL::setDebugOutputStream(*log_stream_);


	if (! plexil_->initialize(xml_interfaces)) {
		throw Exception("Failed to initialize Plexil application");
	}

	if (! plexil_->startInterfaces()) {
		throw Exception("Failed to start Plexil interfaces");
	}

	if (! plexil_->run()) {
		throw Exception("Failed to start Plexil");
	}

	std::string cfg_plan_plx = config->get_string_or_default((cfg_prefix + "plan-plx").c_str(), "");

	if (! cfg_plan_plx.empty()) {
		replace_tokens(cfg_plan_plx);

		pugi::xml_document plan;
		pugi::xml_parse_result parse_result = plan.load_file(cfg_plan_plx.c_str());
		if (parse_result.status != pugi::status_ok) {
			logger->log_error(name(), "Failed to parse plan: %s", parse_result.description());
		} else {
			plexil_->addPlan(&plan);
		}
	} else {
		logger->log_warn(name(), "No plan to execute specified");
	}
}


bool
PlexilExecutiveThread::prepare_finalize_user()
{
	if (! plexil_->stop()) {
		logger->log_error(name(), "Failed to stop Plexil");
	}
	return true;
}

void
PlexilExecutiveThread::finalize()
{
	if (! plexil_->shutdown()) {
		logger->log_error(name(), "Failed to shutdown Plexil");
	}
	delete clock_adapter_;
	delete log_adapter_;
	delete be_adapter_;
}

void
PlexilExecutiveThread::loop()
{
	plexil_->notifyExec();
	plexil_->waitForPlanFinished();
	static PLEXIL::ExecApplication::ApplicationState state = PLEXIL::ExecApplication::APP_SHUTDOWN;
	PLEXIL::ExecApplication::ApplicationState new_state =
	  plexil_->getApplicationState();
	if (new_state != state) {
		logger->log_info(name(), "State changed to %s", plexil_->getApplicationStateName(new_state));
		state = new_state;
	}

	usleep(100000);
}
