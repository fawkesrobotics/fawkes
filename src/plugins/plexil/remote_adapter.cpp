
/***************************************************************************
 *  remote_adapter.cpp - Access Fawkes remotely from PLEXIL
 *
 *  Created: Mon Aug 20 14:41:13 2018
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

#include "remote_adapter.h"

#include "utils.h"

#include <core/threading/mutex_locker.h>
#include <navgraph/navgraph.h>
#include <logging/console.h>
#include <blackboard/remote.h>
#include <config/netconf.h>
#include <utils/time/clock.h>
#include <netcomm/fawkes/client.h>
#include <navgraph/yaml_navgraph.h>
#include <utils/system/fam.h>

#include <InterfaceManager.hh>
#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <Command.hh>
#include <Error.hh>

#include <boost/filesystem.hpp>

using namespace fawkes;
namespace fs = boost::filesystem;


/** @class FawkesRemotePlexilAdapter "remote_adapter.h"
 * Plexil adapter to provide access to the FawkesRemote.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
FawkesRemotePlexilAdapter::FawkesRemotePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface)
: InterfaceAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
FawkesRemotePlexilAdapter::FawkesRemotePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
                                                     pugi::xml_node const xml)
: InterfaceAdapter(execInterface, xml)
{
}

/** Destructor. */
FawkesRemotePlexilAdapter::~FawkesRemotePlexilAdapter()
{
}


/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
FawkesRemotePlexilAdapter::initialize()
{
	std::string cfg_host;
	int         cfg_port;
	bool        cfg_navgraph_allow_multi = true;
	
	clock_   = Clock::instance();
	logger_  = std::make_unique<fawkes::ConsoleLogger>();

	cfg_host = get_xml_config_value(getXml(), "host");
	cfg_port = std::stoi(get_xml_config_value(getXml(), "port"));

	cfg_navgraph_filename_   = get_xml_config_value(getXml(), "navgraph_filename");
	cfg_navgraph_allow_multi = get_xml_config_value(getXml(), "navgraph_allow_multi") == "true";

	client_  = std::make_unique<fawkes::FawkesNetworkClient>(cfg_host.c_str(), cfg_port);

	try {
		logger_->log_info("FawkesRemote", "Connecting to Fawkes at %s:%i", cfg_host.c_str(), cfg_port);
		client_->connect();
	} catch (Exception &e) {
		warn("FawkesRemoteAdapter:initialize: failed to connect to Fawkes: "
		     << e.what_no_backtrace());
		return false;
	}

	logger_->log_info("FawkesRemote", "Mirroring configuration");
	config_  = std::make_unique<fawkes::NetworkConfiguration>(client_.get());
  config_->set_mirror_mode(true);

	logger_->log_info("FawkesRemote", "Accessing blackboard");
	blackboard_ = std::make_unique<fawkes::RemoteBlackBoard>(client_.get());

	if (cfg_navgraph_filename_[0] != '/') {
		cfg_navgraph_filename_ = std::string(CONFDIR) + "/" + cfg_navgraph_filename_;
	}
	fs::path p(cfg_navgraph_filename_);
	p = fs::absolute(p);
	cfg_navgraph_filename_ = p.string();
	logger_->log_info("FawkesRemote", "Loading navgraph file %s", cfg_navgraph_filename_.c_str());
	navgraph_ = load_yaml_navgraph(cfg_navgraph_filename_, cfg_navgraph_allow_multi);

	fs::create_directories(p.parent_path());
	navgraph_fam_ = std::make_unique<fawkes::FileAlterationMonitor>();
	navgraph_fam_->add_filter((std::string("^") + p.filename().string() + "$").c_str());
	navgraph_fam_->watch_dir(p.parent_path().string().c_str());
	navgraph_fam_->add_listener(this);

	navgraph_fam_thread_ = std::thread([this]() {
		                                   while (true) {
			                                   std::unique_lock<std::mutex> lock(this->navgraph_fam_mutex_);
			                                   if (! this->navgraph_fam_) break;
			                                   lock.unlock();
			                                   using namespace std::chrono_literals;
			                                   std::this_thread::sleep_for(1s);
		                                   }
	                                   });

	m_execInterface.setProperty("::Fawkes::Config", config_.get());
	m_execInterface.setProperty("::Fawkes::Clock", clock_);
	m_execInterface.setProperty("::Fawkes::Logger", logger_.get());
	m_execInterface.setProperty("::Fawkes::BlackBoard", blackboard_.get());
	m_execInterface.setProperty("::Fawkes::NavGraph", &navgraph_);

	return true;
}


void
FawkesRemotePlexilAdapter::fam_event(const char *filename, unsigned int mask)
{
	// The file will be ignored from now onwards, re-register
	// if (mask & FAM_IGNORED) {
	// 	boost::filesystem::path p(cfg_navgraph_file_);
	// 	fam_->watch_dir(p.parent_path().string().c_str());
	// }

	if (mask & FAM_DELETE) {
		warn("FawkesRemoteAdapter:fam_event: navgraph file deleted, clearing");
		navgraph_->clear();
		return;
	}

	if (mask & (FAM_MODIFY | FAM_IGNORED)) {
		warn("FawkesRemoteAdapter:fam_event: NavGraph changed on disk, reloading");

		try {
			fawkes::LockPtr<fawkes::NavGraph> new_graph =
			  fawkes::LockPtr<fawkes::NavGraph>(fawkes::load_yaml_navgraph(cfg_navgraph_filename_),
			                                    /* recursive mutex */ true);

			// disable notifications to not trigger them while navgraph is locked
			navgraph_->set_notifications_enabled(false);
			navgraph_.lock();
			**navgraph_ = **new_graph;
			navgraph_.unlock();
			navgraph_->set_notifications_enabled(true);
			navgraph_->notify_of_change();
		} catch (fawkes::Exception &e) {
			warn("FawkesRemoteAdapter:fam_event: loading new graph failed: " << e.what_no_backtrace());
			return;
		} catch (std::runtime_error &e) {
			warn("FawkesRemoteAdapter:fam_event: loading new graph failed: " << e.what());
			return;
		}
	}
}


/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
FawkesRemotePlexilAdapter::start()
{
	return true;
}


/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
FawkesRemotePlexilAdapter::stop()
{
	return true;
}


/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
FawkesRemotePlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
FawkesRemotePlexilAdapter::shutdown()
{
	clock_->finalize();
	client_->disconnect();

	std::unique_lock<std::mutex> lock(navgraph_fam_mutex_);
	navgraph_fam_.reset();
	navgraph_fam_thread_.join();

	return true;
}


/** Perform given command.
 * @param cmd command to execute
 */
void
FawkesRemotePlexilAdapter::executeCommand(PLEXIL::Command* cmd)
{
	/*
	std::string const &name = cmd->getName();

	auto c = commands_.find(name);
	if (c != commands_.end()) {
		c->second(cmd);
	} else {
		warn("FawkesRemoteAdapter:executeCommand: called for unknown"
		     " command " << name);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
	}
	*/
}


/** Abort currently running execution.
 * @param cmd command to abort
 */
void
FawkesRemotePlexilAdapter::invokeAbort(PLEXIL::Command *cmd)
{
}


extern "C" {
	void initFawkesRemoteAdapter() {
		REGISTER_ADAPTER(FawkesRemotePlexilAdapter, "FawkesRemoteAdapter");
	}
}
