
/***************************************************************************
 *  clock_adapter.cpp - PLEXIL adapter for Fawkes' clock
 *
 *  Created: Mon Aug 13 15:49:25 2018
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

#include "clock_adapter.h"

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <State.hh>
#include <StateCacheEntry.hh>

/** @class ClockPlexilTimeAdapter "clock_adapter.h"
 * Plexil adapter to provide time from Fawkes time source.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
ClockPlexilTimeAdapter::ClockPlexilTimeAdapter(PLEXIL::AdapterExecInterface& execInterface)
: TimeAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
ClockPlexilTimeAdapter::ClockPlexilTimeAdapter(PLEXIL::AdapterExecInterface& execInterface, 
                                               pugi::xml_node const xml)
: TimeAdapter(execInterface, xml)
{
}

/** Destructor. */
ClockPlexilTimeAdapter::~ClockPlexilTimeAdapter()
{
}


/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
ClockPlexilTimeAdapter::initialize()
{
	// Automatically register self for time
	clock_ = reinterpret_cast<fawkes::Clock *>(m_execInterface.getProperty("::Fawkes::Clock"));

	PLEXIL::g_configuration->registerLookupInterface("time", this);
	return true;
}


/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
ClockPlexilTimeAdapter::start()
{
	timer_ = new PlexilTimerThread();
	timer_->start();
	
	return true;
}


/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
ClockPlexilTimeAdapter::stop()
{
	timer_->abort_timer();
	timer_->cancel();
	timer_->join();
	delete timer_;

	return true;
}


/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
ClockPlexilTimeAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
ClockPlexilTimeAdapter::shutdown()
{
	return true;
}

/** Immediate lookup of value.
 * @param state state variable to lookup
 * @param cache_entry cache entry for retrieved value
 */
void
ClockPlexilTimeAdapter::lookupNow(PLEXIL::State const &state, PLEXIL::StateCacheEntry &cache_entry)
{
	if (state != PLEXIL::State::timeState()) {
		//warn("TimeAdapter does not implement lookups for state " << state);
		cache_entry.setUnknown();
		return;
	}

	cache_entry.update(getCurrentTime());
}


/** Subscribe to updates for given state.
 * @param state state variable to subscribe for
 */
void
ClockPlexilTimeAdapter::subscribe(const PLEXIL::State& state)
{
	//debugMsg("TimeAdapter:subscribe", " called");
}

/** Unsubscribe from updates.
 * @param state state variable to unsubscribe from
 */
void
ClockPlexilTimeAdapter::unsubscribe(const PLEXIL::State& state)
{
	timer_->abort_timer();
}

/** Set thresholds for subscription.
 * @param state state variable
 * @param hi high value
 * @param lo low value
 */
void
ClockPlexilTimeAdapter::setThresholds(const PLEXIL::State& state, double hi, double lo)
{
	if (state != PLEXIL::State::timeState()) {
		//warn("TimeAdapter does not implement lookups for state " << state);
		return;
	}

	timer_->start_timer(this, hi);
}

/** Set thresholds for subscription.
 * @param state state variable
 * @param hi high value
 * @param lo low value
 */
void
ClockPlexilTimeAdapter::setThresholds(const PLEXIL::State& state, int32_t hi, int32_t lo)
{
	setThresholds(state, (double)hi, (double)lo);
}

void
ClockPlexilTimeAdapter::timer_event()
{
	m_execInterface.notifyOfExternalEvent();
}

/** Get the current time from the operating system.
 * @return A double representing the current time.
 */
double
ClockPlexilTimeAdapter::getCurrentTime() throw (PLEXIL::InterfaceError)
{
	fawkes::Time now(clock_);
	return now.in_sec();
}
