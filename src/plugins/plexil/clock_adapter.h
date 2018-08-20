
/***************************************************************************
 *  clock_adapter.h - PLEXIL adapter for Fawkes' clock
 *
 *  Created: Mon Aug 13 15:13:59 2018
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

#ifndef __PLUGINS_PLEXIL_CLOCK_ADAPTER_H_
#define __PLUGINS_PLEXIL_CLOCK_ADAPTER_H_

#include "timer_thread.h"

#include <TimeAdapter.hh>

#include <utils/time/clock.h>

/**
 * @brief An interface adapter using standard POSIX time facilities
 *        to implement LookupNow and LookupOnChange.
 */
class ClockPlexilTimeAdapter
	: public PLEXIL::TimeAdapter,
	  public PlexilTimerThread::CallbackListener
{
public:
	ClockPlexilTimeAdapter(PLEXIL::AdapterExecInterface& execInterface);
	ClockPlexilTimeAdapter(PLEXIL::AdapterExecInterface& execInterface, 
	                       pugi::xml_node const xml);

	/// @cond DELETED
	ClockPlexilTimeAdapter() = delete;
	ClockPlexilTimeAdapter(const ClockPlexilTimeAdapter &) = delete;
	ClockPlexilTimeAdapter & operator=(const ClockPlexilTimeAdapter &) = delete;
	/// @endcond

	virtual ~ClockPlexilTimeAdapter();
	
	double getCurrentTime() throw (PLEXIL::InterfaceError);

	virtual bool initialize();
	virtual bool start();
	virtual bool stop();
	virtual bool reset();
	virtual bool shutdown();

	virtual void lookupNow(PLEXIL::State const &state, PLEXIL::StateCacheEntry &cacheEntry);
	virtual void subscribe(const PLEXIL::State& state);
	virtual void unsubscribe(const PLEXIL::State& state);
	virtual void setThresholds(const PLEXIL::State& state, double hi, double lo);
	virtual void setThresholds(const PLEXIL::State& state, int32_t hi, int32_t lo);

	virtual void timer_event();

private:
	fawkes::Clock *clock_;
	PlexilTimerThread *timer_;
	
};

extern "C" {
  void initFawkesTimeAdapter();
}

#endif
