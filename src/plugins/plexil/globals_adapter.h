
/***************************************************************************
 *  globals_adapter.h - PLEXIL adapter for global state
 *
 *  Created: Wed Aug 22 11:27:34 2018
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

#ifndef __PLUGINS_PLEXIL_GLOBALS_ADAPTER_H_
#define __PLUGINS_PLEXIL_GLOBALS_ADAPTER_H_

#include <config/config.h>

#include <InterfaceAdapter.hh>
#include <Value.hh>

#include <memory>
#include <functional>
#include <set>

/** Interface adapter to provide logging facilities. */
class GlobalStatePlexilAdapter
	: public PLEXIL::InterfaceAdapter
{
public:
	GlobalStatePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface);
	GlobalStatePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
	                         pugi::xml_node const xml);

	/// @cond DELETED
	GlobalStatePlexilAdapter() = delete;
	GlobalStatePlexilAdapter(const GlobalStatePlexilAdapter &) = delete;
	GlobalStatePlexilAdapter & operator=(const GlobalStatePlexilAdapter &) = delete;
	/// @endcond

	virtual ~GlobalStatePlexilAdapter();

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	virtual void subscribe(const PLEXIL::State& state);
	virtual void unsubscribe(const PLEXIL::State& state);

	virtual void executeCommand(PLEXIL::Command *cmd);
  virtual void invokeAbort(PLEXIL::Command *cmd);

	virtual void lookupNow(PLEXIL::State const &state, PLEXIL::StateCacheEntry &cache_entry);
	
private:
	void global_set_value(PLEXIL::Command* cmd, PLEXIL::ValueType value_type);

private:
	fawkes::Configuration *     config_;
	
	std::map<std::string, std::function<void (PLEXIL::Command*)>> commands_;

	std::map<std::string, std::pair<PLEXIL::ValueType, PLEXIL::Value>> values_;
	std::map<std::string, PLEXIL::State> subscribed_states_;

};

extern "C" {
  void initGlobalState();
}

#endif
