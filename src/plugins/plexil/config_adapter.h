
/***************************************************************************
 *  config_adapter.h - PLEXIL adapter for the configuration
 *
 *  Created: Tue Aug 21 15:41:47 2018
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

#ifndef __PLUGINS_PLEXIL_CONFIG_ADAPTER_H_
#define __PLUGINS_PLEXIL_CONFIG_ADAPTER_H_

#include <config/config.h>
#include <logging/logger.h>

#include <InterfaceAdapter.hh>
#include <Value.hh>
#include <functional>
#include <memory>

/** Interface adapter to provide logging facilities. */
class ConfigurationPlexilAdapter : public PLEXIL::InterfaceAdapter
{
public:
	ConfigurationPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface);
	ConfigurationPlexilAdapter(PLEXIL::AdapterExecInterface &execInterface, pugi::xml_node const xml);

	/// @cond DELETED
	ConfigurationPlexilAdapter()                                   = delete;
	ConfigurationPlexilAdapter(const ConfigurationPlexilAdapter &) = delete;
	ConfigurationPlexilAdapter &operator=(const ConfigurationPlexilAdapter &) = delete;
	/// @endcond

	virtual ~ConfigurationPlexilAdapter();

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	void executeCommand(PLEXIL::Command *cmd);
	void invokeAbort(PLEXIL::Command *cmd);

private:
	void config_get_value(PLEXIL::Command *cmd, PLEXIL::ValueType value_type);
	void config_get_value_or_default(PLEXIL::Command *cmd, PLEXIL::ValueType value_type);
	void config_exists(PLEXIL::Command *cmd);

private:
	fawkes::Configuration *config_;
	fawkes::Logger *       logger_;

	std::map<std::string, std::function<void(PLEXIL::Command *)>> commands_;
};

extern "C" {
void initFawkesConfigurationAdapter();
}

#endif
