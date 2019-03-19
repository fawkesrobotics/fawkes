
/***************************************************************************
 *  navgraph_adapter.h - PLEXIL adapter for the NavGraph
 *
 *  Created: Sun Aug 19 20:49:49 2018
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

#ifndef __PLUGINS_PLEXIL_NAVGRAPH_ADAPTER_H_
#define __PLUGINS_PLEXIL_NAVGRAPH_ADAPTER_H_

#include <core/utils/lockptr.h>

#include <InterfaceAdapter.hh>
#include <Value.hh>
#include <functional>
#include <map>
#include <string>

namespace fawkes {
class BlackBoard;
class Logger;
class NavGraph;
class Position3DInterface;
} // namespace fawkes

/** Interface adapter to provide logging facilities. */
class NavGraphPlexilAdapter : public PLEXIL::InterfaceAdapter
{
public:
	NavGraphPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface);
	NavGraphPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface, pugi::xml_node const xml);

	/// @cond DELETED
	NavGraphPlexilAdapter()                              = delete;
	NavGraphPlexilAdapter(const NavGraphPlexilAdapter &) = delete;
	NavGraphPlexilAdapter &operator=(const NavGraphPlexilAdapter &) = delete;
	/// @endcond

	virtual ~NavGraphPlexilAdapter();

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	void executeCommand(PLEXIL::Command *cmd);
	void invokeAbort(PLEXIL::Command *cmd);

private:
	void navgraph_get_nodes(PLEXIL::Command *cmd);
	void navgraph_cost_to(PLEXIL::Command *cmd);
	void navgraph_cost_between(PLEXIL::Command *cmd);

private:
	fawkes::Logger *             logger_;
	fawkes::BlackBoard *         blackboard_;
	fawkes::Position3DInterface *pose_if_;

	fawkes::LockPtr<fawkes::NavGraph> navgraph_;

	std::map<std::string, std::function<void(PLEXIL::Command *)>> commands_;
};

extern "C" {
void initNavGraphAdapter();
}

#endif
