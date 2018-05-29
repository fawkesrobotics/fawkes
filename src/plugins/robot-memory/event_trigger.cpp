/***************************************************************************
 *  event_trigger.cpp - Class for handling EventTriggers (such as a subscriber)
 *    
 *
 *  Created: 7:03:38 PM 2016
 *  Copyright  2016  Frederik Zwilling
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

#include "event_trigger.h"
#include "event_trigger_manager.h"
#include <core/exception.h>


/** @class EventTrigger  event_trigger.h
 * Class holding all information about an EventTrigger
 * @author Frederik Zwilling
 */

/** Constructor.
 * @param oplog_query The Query on the oplog, which defines which database update invokes the trigger
 * @param ns namespace of the trigger, format db.collection
 * @param callback Reference to callback function
 */
EventTrigger::EventTrigger(mongo::Query oplog_query, const std::string& ns,
                           const boost::function<void (mongo::BSONObj)> &callback)
	: oplog_query(oplog_query), ns(ns), ns_db(EventTriggerManager::get_db_name(ns)),
	  callback(callback)
{
	if (ns_db == "") {
		throw fawkes::Exception("Invalid namespace, does not reference database");
	}
}

EventTrigger::~EventTrigger(){}

