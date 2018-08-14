
/***************************************************************************
 *  log_adapter.h - PLEXIL adapter for logging
 *
 *  Created: Tue Aug 14 13:38:18 2018
 *  Copyright  2006-2018  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __PLUGINS_PLEXIL_LOG_ADAPTER_H_
#define __PLUGINS_PLEXIL_LOG_ADAPTER_H_

#include <InterfaceAdapter.hh>

#include <logging/logger.h>

/** Interface adapter to provide logging facilities. */
class LoggingPlexilAdapter
	: public PLEXIL::InterfaceAdapter
{
public:
	LoggingPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface);
	LoggingPlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
	                     pugi::xml_node const xml);

	/// @cond DELETED
	LoggingPlexilAdapter() = delete;
	LoggingPlexilAdapter(const LoggingPlexilAdapter &) = delete;
	LoggingPlexilAdapter & operator=(const LoggingPlexilAdapter &) = delete;
	/// @endcond

	virtual ~LoggingPlexilAdapter();

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	void executeCommand(PLEXIL::Command *cmd);
  void invokeAbort(PLEXIL::Command *cmd);

private:
	fawkes::Logger *logger_;
	
};

extern "C" {
  void initFawkesLoggingAdapter();
}

#endif // POSIX_TIME_ADAPTER_H
