
/***************************************************************************
 *  thread_adapter.cpp - PLEXIL thread name adapter
 *
 *  Created: Wed Aug 15 12:35:55 2018
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

#include "thread_adapter.h"

#include <AdapterConfiguration.hh>
#include <AdapterExecInterface.hh>
#include <AdapterFactory.hh>

#include <core/threading/thread.h>
#include <cstring>

/** @class ThreadNamePlexilAdapter "log_adapter.h"
 * Plexil adapter to provide logging facilities.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 */
ThreadNamePlexilAdapter::ThreadNamePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface)
: InterfaceAdapter(execInterface)
{
}

/** Constructor from configuration XML.
 * @param execInterface Reference to the parent AdapterExecInterface object.
 * @param xml A const reference to the XML element describing this adapter
 * @note The instance maintains a shared pointer to the XML.
 */
ThreadNamePlexilAdapter::ThreadNamePlexilAdapter(PLEXIL::AdapterExecInterface& execInterface, 
                                               pugi::xml_node const xml)
: InterfaceAdapter(execInterface, xml)
{
}

/** Destructor. */
ThreadNamePlexilAdapter::~ThreadNamePlexilAdapter()
{
}


/** Initialize adapter.
 * @return true if initialization was successful, false otherwise.
 */
bool
ThreadNamePlexilAdapter::initialize()
{
	return true;
}


/** Start adapter.
 * @return true if starting was successful, false otherwise.
 */
bool
ThreadNamePlexilAdapter::start()
{
	std::string name;

	pugi::xml_node config = getXml();
	pugi::xml_attribute xml_attr = config.attribute("name");
	if (xml_attr) {
		name = xml_attr.value();
	} else {
		for (const auto &c : config.children()) {
			if (strcmp(c.name(), "Parameter") == 0) {
				pugi::xml_attribute xml_key_attr = c.attribute("key");
				if (xml_key_attr && strcmp(xml_key_attr.value(), "name") == 0) {
					name = c.text().get();
				}
			}
		}
	}

	if (! name.empty()) {
		fawkes::Thread::current_thread_name(name);
	} else if (fawkes::Thread::current_thread_name() == "") {
		fawkes::Thread::current_thread_name("PlexilExecutive");
	}
	return true;
}


/** Stop adapter.
 * @return true if successful, false otherwise.
 */
bool
ThreadNamePlexilAdapter::stop()
{
	return true;
}


/** Reset adapter.
 * @return true if successful, false otherwise.
 */
bool
ThreadNamePlexilAdapter::reset()
{
	return true;
}

/** Shut adapter down.
 * @return true if successful, false otherwise.
 */
bool
ThreadNamePlexilAdapter::shutdown()
{
	return true;
}

extern "C" {
	void initThreadNameAdapter() {
		REGISTER_ADAPTER(ThreadNamePlexilAdapter, "ThreadNameAdapter");
	}
}
