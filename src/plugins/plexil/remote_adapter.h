
/***************************************************************************
 *  remote_adapter.h - Access Fawkes remotely from PLEXIL
 *
 *  Created: Mon Aug 20 12:58:05 2018
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

#ifndef __PLUGINS_PLEXIL_REMOTE_ADAPTER_H_
#define __PLUGINS_PLEXIL_REMOTE_ADAPTER_H_

#include <core/utils/lockptr.h>
#include <utils/system/fam.h>

#include <InterfaceAdapter.hh>
#include <Value.hh>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace fawkes {
class BlackBoard;
class Logger;
class NavGraph;
class Clock;
class NetworkConfiguration;
class FawkesNetworkClient;
class FileAlterationMonitor;
} // namespace fawkes

/** Interface adapter to provide logging facilities. */
class FawkesRemotePlexilAdapter : public PLEXIL::InterfaceAdapter, public fawkes::FamListener
{
public:
	FawkesRemotePlexilAdapter(PLEXIL::AdapterExecInterface &execInterface);
	FawkesRemotePlexilAdapter(PLEXIL::AdapterExecInterface &execInterface, pugi::xml_node const xml);

	/// @cond DELETED
	FawkesRemotePlexilAdapter()                                  = delete;
	FawkesRemotePlexilAdapter(const FawkesRemotePlexilAdapter &) = delete;
	FawkesRemotePlexilAdapter &operator=(const FawkesRemotePlexilAdapter &) = delete;
	/// @endcond

	virtual ~FawkesRemotePlexilAdapter();

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	void executeCommand(PLEXIL::Command *cmd);
	void invokeAbort(PLEXIL::Command *cmd);

	virtual void fam_event(const char *filename, unsigned int mask);

private:
	std::string cfg_navgraph_filename_;

	fawkes::Clock *                                clock_;
	std::unique_ptr<fawkes::FawkesNetworkClient>   client_;
	std::unique_ptr<fawkes::Logger>                logger_;
	std::unique_ptr<fawkes::BlackBoard>            blackboard_;
	std::unique_ptr<fawkes::NetworkConfiguration>  config_;
	fawkes::LockPtr<fawkes::NavGraph>              navgraph_;
	std::unique_ptr<fawkes::FileAlterationMonitor> navgraph_fam_;
	std::mutex                                     navgraph_fam_mutex_;
	std::thread                                    navgraph_fam_thread_;

	std::map<std::string, std::function<void(PLEXIL::Command *)>> commands_;
};

extern "C" {
void initNavGraphAdapter();
}

#endif
