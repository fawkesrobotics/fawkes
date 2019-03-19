
/***************************************************************************
 *  blackboard_adapter.h - PLEXIL adapter for Fawkes' blackboard
 *
 *  Created: Sun Feb 17 12:46:23 2019 +0100
 *  Copyright  2006-2019  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PLEXIL_BLACKBOARD_ADAPTER_H_
#define __PLUGINS_PLEXIL_BLACKBOARD_ADAPTER_H_

#include <blackboard/blackboard.h>
#include <blackboard/interface_listener.h>
#include <logging/logger.h>

#include <InterfaceAdapter.hh>
#include <Value.hh>
#include <functional>
#include <map>
#include <mutex>
#include <string>

namespace fawkes {
class Interface;
}

/**
 * @brief An interface adapter using standard POSIX time facilities
 *        to implement LookupNow and LookupOnChange.
 */
class BlackboardPlexilAdapter : public PLEXIL::InterfaceAdapter,
                                public fawkes::BlackBoardInterfaceListener
{
public:
	BlackboardPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface);
	BlackboardPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface, pugi::xml_node const xml);

	/// @cond DELETED
	BlackboardPlexilAdapter()                                = delete;
	BlackboardPlexilAdapter(const BlackboardPlexilAdapter &) = delete;
	BlackboardPlexilAdapter &operator=(const BlackboardPlexilAdapter &) = delete;
	/// @endcond

	virtual ~BlackboardPlexilAdapter();

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	virtual void lookupNow(PLEXIL::State const &state, PLEXIL::StateCacheEntry &cacheEntry);

	virtual void subscribe(const PLEXIL::State &state);
	virtual void unsubscribe(const PLEXIL::State &state);

	virtual void executeCommand(PLEXIL::Command *cmd);

private:
	virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

	void bb_open_for_reading(PLEXIL::Command *cmd);
	void bb_close(PLEXIL::Command *cmd);
	void bb_read(PLEXIL::Command *cmd);
	void bb_read_all(PLEXIL::Command *cmd);
	void bb_print(PLEXIL::Command *cmd);

private:
	fawkes::Logger *    logger_;
	fawkes::BlackBoard *blackboard_;

	std::mutex                                 ifs_read_mutex_;
	std::map<std::string, fawkes::Interface *> ifs_read_;

	std::map<std::string, std::function<void(PLEXIL::Command *)>> commands_;

	std::multimap<std::string, PLEXIL::State> subscribed_states_;
};

extern "C" {
void initFawkesBlackboardAdapter();
}

#endif
