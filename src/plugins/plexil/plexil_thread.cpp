
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
#include "protobuf_adapter.h"
#ifdef HAVE_NAVGRAPH
#  include "navgraph_access_thread.h"
#  include "navgraph_adapter.h"
#endif
#include "utils.h"

#include <core/threading/mutex_locker.h>
#include <utils/sub_process/proc.h>

#include <ExecApplication.hh>
#include <Debug.hh>
#include <InterfaceSchema.hh>
#include <InterfaceManager.hh>
#include <AdapterConfiguration.hh>
#include <pugixml.hpp>

#include <fstream>
#include <cstring>
#include <numeric>
#include <boost/filesystem.hpp>

using namespace fawkes;
namespace fs = boost::filesystem;
// for C++17 could be:
// namespace fs = std::filesystem;

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

	clock_adapter_    = new PLEXIL::ConcreteAdapterFactory<ClockPlexilTimeAdapter>("FawkesTime");
	log_adapter_      = new PLEXIL::ConcreteAdapterFactory<LoggingPlexilAdapter>("FawkesLogging");
	be_adapter_       = new PLEXIL::ConcreteAdapterFactory<BehaviorEnginePlexilAdapter>("BehaviorEngine");
	thread_adapter_   = new PLEXIL::ConcreteAdapterFactory<ThreadNamePlexilAdapter>("ThreadName");
	protobuf_adapter_ = new PLEXIL::ConcreteAdapterFactory<ProtobufCommPlexilAdapter>("ProtobufComm");

	pugi::xml_document xml_config;
	pugi::xml_node xml_interfaces =
	  xml_config.append_child(PLEXIL::InterfaceSchema::INTERFACES_TAG());

	add_plexil_interface_configs(xml_interfaces, cfg_adapters,
	                             PLEXIL::InterfaceSchema::ADAPTER_TAG(),
	                             PLEXIL::InterfaceSchema::ADAPTER_TYPE_ATTR());
	add_plexil_interface_configs(xml_interfaces, cfg_listeners,
	                             PLEXIL::InterfaceSchema::LISTENER_TAG(),
	                             PLEXIL::InterfaceSchema::LISTENER_TYPE_ATTR());

	auto navgraph_adapter_config = std::find_if(cfg_adapters.begin(), cfg_adapters.end(),
	                                            [](const auto &entry) {
		                                            return entry.second.type == "NavGraph";
	                                            });
	if (navgraph_adapter_config != cfg_adapters.end()) {
#ifdef HAVE_NAVGRAPH
		navgraph_access_thread_ = new PlexilNavgraphAccessThread();
		thread_collector->add(navgraph_access_thread_);
		navgraph_ = navgraph_access_thread_->get_navgraph();
		PLEXIL::g_manager->setProperty("::Fawkes::NavGraph", &navgraph_);
		navgraph_adapter_ = new PLEXIL::ConcreteAdapterFactory<NavGraphPlexilAdapter>("NavGraph");
#else
		throw Exception("NavGraph adapter configured, "
		                "but navgraph library not available at compile time");
#endif
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

	cfg_plan_ple_           = config->get_string_or_default((cfg_prefix + "plan/ple").c_str(), "");
	cfg_plan_plx_           = config->get_string_or_default((cfg_prefix + "plan/plx").c_str(), "");
	cfg_plan_auto_compile_  = config->get_bool_or_default((cfg_prefix + "plan/compilation/enable").c_str(), false);
	cfg_plan_force_compile_ = config->get_bool_or_default((cfg_prefix + "plan/compilation/force").c_str(), false);

	fs::path ple_path, plx_path;

	replace_tokens(cfg_plan_ple_);
	replace_tokens(cfg_plan_plx_);

	if (cfg_plan_plx_.empty()) {
		if (! cfg_plan_auto_compile_) {
			throw Exception("PLX not configured and auto-compile disabled");
		}
		if (cfg_plan_ple_.empty()) {
			throw Exception("Neither PLX nor PLE configured");
		}
		ple_path = cfg_plan_ple_;
		if (! fs::exists(ple_path)) {
			throw Exception("PLE configured, but file does not exist");
		}
		if (ple_path.extension() != ".ple") {
			throw Exception("Unknown PLE extension %s, expected .ple", ple_path.extension().string().c_str());
		}
		plx_path = fs::path{ple_path}.replace_extension(".plx");
		cfg_plan_plx_ = plx_path.string();
		if (cfg_plan_plx_.empty()) {
			throw Exception("Failed to automatically determine PLX");
		}
	}

	if (cfg_plan_auto_compile_) {
		if (! fs::exists(cfg_plan_plx_) || cfg_plan_force_compile_ ||
		    fs::last_write_time(cfg_plan_plx_) < fs::last_write_time(cfg_plan_ple_))
		{
			logger->log_info(name(), "Compiling %s", cfg_plan_ple_.c_str());
			plexil_compile(cfg_plan_ple_, cfg_plan_plx_);
		}
	} else {
		if (! fs::exists(cfg_plan_plx_)) {
			throw Exception("PLX %s does not exist and auto-compile disabled", cfg_plan_plx_.c_str());
		}
		if (fs::last_write_time(cfg_plan_plx_) < fs::last_write_time(cfg_plan_ple_))
		{
			logger->log_warn(name(), "PLX older than PLE, but auto-compile disabled");
		}
	}

	plan_plx_.reset(new pugi::xml_document);
	pugi::xml_parse_result parse_result = plan_plx_->load_file(cfg_plan_plx_.c_str());
	if (parse_result.status != pugi::status_ok) {
		throw Exception("Failed to parse plan '%s': %s",
		                cfg_plan_plx_.c_str(), parse_result.description());
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
	plexil_->notifyExec();
}


bool
PlexilExecutiveThread::prepare_finalize_user()
{
	if (! plexil_->stop()) {
		logger->log_error(name(), "Failed to stop Plexil");
	}
	plexil_->notifyExec();
	return true;
}

void
PlexilExecutiveThread::finalize()
{
	if (! plexil_->shutdown()) {
		logger->log_error(name(), "Failed to shutdown Plexil");
	}
	PLEXIL::g_configuration->clearAdapterRegistry();
	plexil_->waitForShutdown();

	// We really should do a reset here, killing off the ExecApplication instance.
	// However, the executive crashes in a state cache destructor if there is any
	// active wait (or probably any active LookupOnChange, as here on time).
	// Therefore, we accept this memleak here under the assumption, that we do not
	// frequently reload the plexil plugin. This at least avoids the segfaut on quit.
	plexil_.release();
	//plexil_.reset();
	log_stream_.reset();
	log_buffer_.reset();
	plan_plx_.reset();
  delete clock_adapter_;
	delete log_adapter_;
	delete be_adapter_;
	delete thread_adapter_;
	delete protobuf_adapter_;
#ifdef HAVE_NAVGRAPH
	if (navgraph_) {
		delete navgraph_adapter_;
		navgraph_.clear();
		thread_collector->remove(navgraph_access_thread_);
		delete navgraph_access_thread_;
	}
#endif
}

void
PlexilExecutiveThread::loop()
{
	//plexil_->notifyExec();
	//plexil_->waitForPlanFinished();
	static PLEXIL::ExecApplication::ApplicationState state = PLEXIL::ExecApplication::APP_SHUTDOWN;
	PLEXIL::ExecApplication::ApplicationState new_state =
	  plexil_->getApplicationState();
	if (new_state != state) {
		logger->log_info(name(), "State changed to %s", plexil_->getApplicationStateName(new_state));
		state = new_state;
	}

	using namespace std::chrono_literals;
	std::this_thread::sleep_for(500ms);
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
			} else if (what == "verbatim-xml") {
				logger->log_warn(name(), "Parsing verbatim");
				pugi::xml_parse_result parse_result =
				  cfg_adapters[id].verbatim.load_string(cfg_item->get_string().c_str());
				if (parse_result.status != pugi::status_ok) {
					throw Exception("Failed to parse verbatim-xml for '%s': %s",
					                cfg_adapters[id].type.c_str(), parse_result.description());
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
		if (a.verbatim && a.verbatim.children().begin() != a.verbatim.children().end()) {
			for (const auto &child: a.verbatim.children()) {
				xml_adapter.append_copy(child);
			}
		}
	}
}

void
PlexilExecutiveThread::plexil_compile(const std::string& ple_file, const std::string& plx_file)
{
	std::vector<std::string> argv{"plexilc", "-o", plx_file, ple_file};
	std::string command_line =
	  std::accumulate(std::next(argv.begin()), argv.end(), argv.front(),
	                  [](std::string &s, const std::string &a) { return s + " " + a; });
	logger->log_debug(name(), "Compiler command: %s", command_line.c_str());

	SubProcess proc("plexilc", "plexilc", argv, {}, logger);
	using namespace std::chrono_literals;
	auto compile_start = std::chrono::system_clock::now();
	auto now = std::chrono::system_clock::now();
	do {
		proc.check_proc();
		if (! proc.alive()) {
			if (proc.exit_status() != 0) {
				throw Exception("Plexil compilation failed, check log for messages.");
			} else {
				break;
			}
		}
		now = std::chrono::system_clock::now();
		std::this_thread::sleep_for(500ms);
	} while (now < compile_start + 30s);
	if (proc.alive()) {
		proc.kill(SIGINT);
		throw Exception("Plexil compilation timeout after 30s");
	}
}
