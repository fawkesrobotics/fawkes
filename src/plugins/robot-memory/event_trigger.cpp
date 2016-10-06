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

EventTrigger::EventTrigger(mongo::Query oplog_query, std::string oplog_collection,
  const boost::function<void (mongo::BSONObj)> &callback)
{
  this->oplog_query = oplog_query;
  this->oplog_collection = oplog_collection;
  this->callback = callback;
}

EventTrigger::~EventTrigger(){}

