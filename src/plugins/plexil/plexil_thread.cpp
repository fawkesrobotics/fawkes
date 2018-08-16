
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
#include "thread_adapter.h"

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

	bool cfg_print_xml =
	  config->get_bool_or_default((cfg_prefix + "debug/print-xml").c_str(), false);

	std::map<std::string, plexil_interface_config> cfg_adapters =
	  read_plexil_interface_configs(cfg_prefix + "adapters/");

	std::map<std::string, plexil_interface_config> cfg_listeners =
	  read_plexil_interface_configs(cfg_prefix + "listeners/");

	for (const auto &a_item : cfg_adapters) {
		const auto &a = a_item.second;
		if (a.type == "Utility") {
			logger->log_warn(name(), "Utility adapter configured, consider using FawkesLogging instead");
		} else if (a.type == "OSNativeTime") {
			logger->log_warn(name(), "OSNativeTime adapter configured, consider using FawkesTime instead");
		}
	}

	plexil_.reset(new PLEXIL::ExecApplication);

	PLEXIL::g_manager->setProperty("::Fawkes::Config", config);
	PLEXIL::g_manager->setProperty("::Fawkes::Clock", clock);
	PLEXIL::g_manager->setProperty("::Fawkes::Logger", logger);
	PLEXIL::g_manager->setProperty("::Fawkes::BlackBoard", blackboard);

	clock_adapter_  = new PLEXIL::ConcreteAdapterFactory<ClockPlexilTimeAdapter>("FawkesTime");
	log_adapter_    = new PLEXIL::ConcreteAdapterFactory<LoggingPlexilAdapter>("FawkesLogging");
	be_adapter_     = new PLEXIL::ConcreteAdapterFactory<BehaviorEnginePlexilAdapter>("BehaviorEngine");
	thread_adapter_ = new PLEXIL::ConcreteAdapterFactory<ThreadNamePlexilAdapter>("ThreadName");

	pugi::xml_document xml_config;
	pugi::xml_node xml_interfaces =
	  xml_config.append_child(PLEXIL::InterfaceSchema::INTERFACES_TAG());

	add_plexil_interface_configs(xml_interfaces, cfg_adapters,
	                             PLEXIL::InterfaceSchema::ADAPTER_TAG(),
	                             PLEXIL::InterfaceSchema::ADAPTER_TYPE_ATTR());
	add_plexil_interface_configs(xml_interfaces, cfg_listeners,
	                             PLEXIL::InterfaceSchema::LISTENER_TAG(),
	                             PLEXIL::InterfaceSchema::LISTENER_TYPE_ATTR());

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

	if (config->get_bool_or_default((cfg_prefix + "debug/enable").c_str(), false)) {
		std::vector<std::string> debug_markers =
		  config->get_strings_or_defaults((cfg_prefix + "debug/markers").c_str(), {});

		std::stringstream dbg_config;
		for (const auto &m : debug_markers) {
			dbg_config << m << std::endl;
		}
		PLEXIL::readDebugConfigStream(dbg_config);
	}

	log_buffer_.reset(new PlexilLogStreamBuffer(logger));
	log_stream_.reset(new std::ostream(&*log_buffer_));
	PLEXIL::setDebugOutputStream(*log_stream_);


	if (! plexil_->initialize(xml_interfaces)) {
		throw Exception("Failed to initialize Plexil application");
	}

	cfg_plan_plx_ = config->get_string_or_default((cfg_prefix + "plan-plx").c_str(), "");

	if (! cfg_plan_plx_.empty()) {
		replace_tokens(cfg_plan_plx_);

		plan_plx_.reset(new pugi::xml_document);
		pugi::xml_parse_result parse_result = plan_plx_->load_file(cfg_plan_plx_.c_str());
		if (parse_result.status != pugi::status_ok) {
			throw Exception("Failed to parse plan '%s': %s",
			                cfg_plan_plx_.c_str(), parse_result.description());
		}
	} else {
		logger->log_warn(name(), "No plan to execute specified");
	}
}

void
PlexilExecutiveThread::once()
{
	if (! plexil_->startInterfaces()) {
		throw Exception("Failed to start Plexil interfaces");
	}
	if (! plexil_->run()) {
		throw Exception("Failed to start Plexil");
	}

	plexil_->addPlan(&*plan_plx_);
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
	plexil_.reset();
	log_stream_.reset();
	log_buffer_.reset();
	plan_plx_.reset();
  delete clock_adapter_;
	delete log_adapter_;
	delete be_adapter_;
	delete thread_adapter_;
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


// Parse adapter configurations
std::map<std::string, PlexilExecutiveThread::plexil_interface_config>
PlexilExecutiveThread::read_plexil_interface_configs(const std::string& config_prefix)
{
	std::map<std::string, plexil_interface_config> cfg_adapters;

	std::unique_ptr<Configuration::ValueIterator>
	  cfg_item{config->search(config_prefix)};
	while (cfg_item->next()) {
		std::string path = cfg_item->path();

		std::string::size_type start_pos = config_prefix.size();
		std::string::size_type slash_pos = path.find("/", start_pos + 1);
		if (slash_pos != std::string::npos) {
			std::string id = path.substr(start_pos, slash_pos - start_pos);

			start_pos = slash_pos + 1;
			slash_pos = path.find("/", start_pos);
			std::string what = path.substr(start_pos, slash_pos - start_pos);

			if (what == "type") {
				cfg_adapters[id].type = cfg_item->get_string();
			} else if (what == "attr") {
				start_pos = slash_pos + 1;
				slash_pos = path.find("/", start_pos);
				std::string key = path.substr(start_pos, slash_pos - start_pos);
				cfg_adapters[id].attr[key] = cfg_item->get_as_string();
			} else if (what == "args") {
				start_pos = slash_pos + 1;
				slash_pos = path.find("/", start_pos);
				std::string key = path.substr(start_pos, slash_pos - start_pos);
				cfg_adapters[id].args[key] = cfg_item->get_as_string();
			} else if (what == "verbatim-args") {
				start_pos = slash_pos + 1;
				slash_pos = path.find("/", start_pos);
				std::string verb_id = path.substr(start_pos, slash_pos - start_pos);

				start_pos = slash_pos + 1;
				slash_pos = path.find("/", start_pos);
				std::string verb_what = path.substr(start_pos, slash_pos - start_pos);

				if (verb_what == "tag") {
					cfg_adapters[id].verbatim_args[verb_id].tag = cfg_item->get_as_string();
				} else if (verb_what == "text") {
					cfg_adapters[id].verbatim_args[verb_id].has_text = true;
					cfg_adapters[id].verbatim_args[verb_id].text = cfg_item->get_as_string();
				} else if (verb_what == "attr") {
					start_pos = slash_pos + 1;
					slash_pos = path.find("/", start_pos);
					std::string verb_key = path.substr(start_pos, slash_pos - start_pos);
					cfg_adapters[id].verbatim_args[verb_id].attr[verb_key] = cfg_item->get_as_string();
				}
			}
		}
	}
	return cfg_adapters;
}

	// Add adapter configurations to Plexil interface XML config
void
PlexilExecutiveThread::add_plexil_interface_configs(
  pugi::xml_node &parent,
  const std::map<std::string, PlexilExecutiveThread::plexil_interface_config> &configs,
  const char* tag_name, const char* type_attr_name)
{
	for (const auto &a_item : configs) {
		const auto &a = a_item.second;
		pugi::xml_node xml_adapter = parent.append_child(tag_name);
		xml_adapter.append_attribute(type_attr_name).set_value(a.type.c_str());
		for (const auto &attr : a.attr) {
			xml_adapter.append_attribute(attr.first.c_str()).set_value(attr.second.c_str());
		}
		for (const auto &arg : a.args) {
			pugi::xml_node xml_adapter_arg = xml_adapter.append_child("Parameter");
			xml_adapter_arg.append_attribute("key").set_value(arg.first.c_str());
			xml_adapter_arg.text().set(arg.second.c_str());
		}
		for (const auto &arg : a.verbatim_args) {
			const auto &varg = arg.second;
			pugi::xml_node xml_adapter_arg = xml_adapter.append_child(varg.tag.c_str());
			for (const auto &attr: varg.attr) {
				xml_adapter_arg.append_attribute(attr.first.c_str()).set_value(attr.second.c_str());
			}
			if (varg.has_text) {
				xml_adapter_arg.text().set(varg.text.c_str());
			}
		}
	}
}
