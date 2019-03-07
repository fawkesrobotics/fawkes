
/***************************************************************************
 *  navgraph_adapter.cpp - PLEXIL adapter for the Behavior Engine
 *
 *  Created: Sun Aug 19 20:57:53 2018
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

#include "navgraph_adapter.h"

#include "utils.h"

#include <blackboard/blackboard.h>
#include <core/threading/mutex_locker.h>
#include <interfaces/Position3DInterface.h>
#include <logging/logger.h>
#include <navgraph/navgraph.h>

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>
#include <ArrayImpl.hh>
#include <Command.hh>
#include <Error.hh>
#include <algorithm>

using namespace fawkes;

/** @class NavGraphPlexilAdapter "navgraph_adapter.h"
 * Plexil adapter to provide access to the NavGraph.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
NavGraphPlexilAdapter::NavGraphPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface)
: InterfaceAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
NavGraphPlexilAdapter::NavGraphPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface,
                                             pugi::xml_node const          xml)
: InterfaceAdapter(execInterface, xml)
{
}

/** Destructor. */
NavGraphPlexilAdapter::~NavGraphPlexilAdapter()
{
}

/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
NavGraphPlexilAdapter::initialize()
{
	logger_ = reinterpret_cast<fawkes::Logger *>(m_execInterface.getProperty("::Fawkes::Logger"));
	blackboard_ =
	  reinterpret_cast<fawkes::BlackBoard *>(m_execInterface.getProperty("::Fawkes::BlackBoard"));
	navgraph_ = *reinterpret_cast<fawkes::LockPtr<fawkes::NavGraph> *>(
	  m_execInterface.getProperty("::Fawkes::NavGraph"));

	try {
		pose_if_ = blackboard_->open_for_reading<Position3DInterface>("Pose");
	} catch (Exception &e) {
		logger_->log_error("PlexilBE", "Failed to open pose interface: %s", e.what_no_backtrace());
		return false;
	}

	namespace p = std::placeholders;
	commands_   = {
    {"navgraph_get_nodes", std::bind(&NavGraphPlexilAdapter::navgraph_get_nodes, this, p::_1)},
    {"navgraph_cost_to", std::bind(&NavGraphPlexilAdapter::navgraph_cost_to, this, p::_1)},
    {"navgraph_cost_between",
     std::bind(&NavGraphPlexilAdapter::navgraph_cost_between, this, p::_1)},
  };

	for (const auto &c : commands_) {
		PLEXIL::g_configuration->registerCommandInterface(c.first, this);
	}

	return true;
}

/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
NavGraphPlexilAdapter::start()
{
	return true;
}

/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
NavGraphPlexilAdapter::stop()
{
	return true;
}

/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
NavGraphPlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
NavGraphPlexilAdapter::shutdown()
{
	blackboard_->close(pose_if_);
	return true;
}

/** Perform given command.
 * @param cmd command to execute
 */
void
NavGraphPlexilAdapter::executeCommand(PLEXIL::Command *cmd)
{
	std::string const &name = cmd->getName();

	auto c = commands_.find(name);
	if (c != commands_.end()) {
		c->second(cmd);
	} else {
		warn("NavGraphAdapter:executeCommand: called for unknown"
		     " command "
		     << name);
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
	}
}

void
NavGraphPlexilAdapter::navgraph_get_nodes(PLEXIL::Command *cmd)
{
	fawkes::MutexLocker              lock(navgraph_.objmutex_ptr());
	const std::vector<NavGraphNode> &nodes = navgraph_->nodes();
	PLEXIL::StringArray              array(nodes.size());
	for (size_t i = 0; i < nodes.size(); ++i) {
		array.setElement(i, nodes[i].name());
	}
	m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(array));
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

void
NavGraphPlexilAdapter::navgraph_cost_to(PLEXIL::Command *cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (!verify_args(args,
	                 "NavGraphAdapter:navgraph_cost_to",
	                 {{"target_node", PLEXIL::STRING_TYPE}})) {
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string target_node;
	args[0].getValue(target_node);

	if (!pose_if_->has_writer()) {
		warn("NavGraphAdapter:navgraph_cost_to:"
		     << " Cannot determine distance without pose provider (no writer for" << pose_if_->uid()
		     << ")");
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	fawkes::MutexLocker lock(navgraph_.objmutex_ptr());

	pose_if_->read();

	float cost = 0;

	NavGraphNode closest =
	  navgraph_->closest_node(pose_if_->translation(0), pose_if_->translation(1));
	NavGraphNode src_node("Pose", pose_if_->translation(0), pose_if_->translation(1));
	cost += navgraph_->cost(src_node, closest);

	if (closest.name() != target_node) {
		try {
			NavGraphPath path = navgraph_->search_path(closest.name(), target_node);
			cost += path.cost();
		} catch (Exception &e) {
			warn("NavGraphAdapter:navgraph_cost_to:"
			     << " Failed to generate path from " << closest.name() << " to " << target_node << ": "
			     << e.what_no_backtrace());
			m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
			m_execInterface.notifyOfExternalEvent();
			return;
		}
	}

	m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(cost));
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

void
NavGraphPlexilAdapter::navgraph_cost_between(PLEXIL::Command *cmd)
{
	std::vector<PLEXIL::Value> const &args = cmd->getArgValues();
	if (!verify_args(args,
	                 "NavGraphAdapter:navgraph_cost_to",
	                 {{"source_node", PLEXIL::STRING_TYPE}, {"target_node", PLEXIL::STRING_TYPE}})) {
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	std::string source_node;
	std::string target_node;
	args[0].getValue(source_node);
	args[1].getValue(target_node);

	fawkes::MutexLocker lock(navgraph_.objmutex_ptr());

	float cost = 0.;

	try {
		NavGraphPath path = navgraph_->search_path(source_node, target_node);
		cost              = path.cost();
	} catch (Exception &e) {
		warn("NavGraphAdapter:navgraph_cost_between:"
		     << " Failed to generate path from " << source_node << " to " << target_node << ": "
		     << e.what_no_backtrace());
		m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_FAILED);
		m_execInterface.notifyOfExternalEvent();
		return;
	}

	m_execInterface.handleCommandReturn(cmd, PLEXIL::Value(cost));
	m_execInterface.handleCommandAck(cmd, PLEXIL::COMMAND_SUCCESS);
	m_execInterface.notifyOfExternalEvent();
}

/** Abort currently running execution.
 * @param cmd command to abort
 */
void
NavGraphPlexilAdapter::invokeAbort(PLEXIL::Command *cmd)
{
}

extern "C" {
void
initNavGraphAdapter()
{
	REGISTER_ADAPTER(NavGraphPlexilAdapter, "NavGraphAdapter");
}
}
