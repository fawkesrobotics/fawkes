/***************************************************************************
 *  event_trigger.h - Class for handling EventTriggers (such as a subscriber)
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

#ifndef FAWKES_SRC_PLUGINS_ROBOT_MEMORY_EVENT_TRIGGER_H_
#define FAWKES_SRC_PLUGINS_ROBOT_MEMORY_EVENT_TRIGGER_H_

#include <mongo/client/dbclient.h>
#include <boost/function.hpp>

///typedef for shorter type description
typedef std::unique_ptr<mongo::DBClientCursor> QResCursor;


class EventTrigger
{
  /// Information access for Manager
  friend class EventTriggerManager;

  public:
    EventTrigger(mongo::Query oplog_query, const std::string &ns,
                 const boost::function<void (mongo::BSONObj)> &callback);
    virtual ~EventTrigger();

  private:
    mongo::Query oplog_query;
    std::string ns;
    std::string ns_db;
    QResCursor oplog_cursor;
    boost::function<void (mongo::BSONObj)> callback;
};

#endif /* FAWKES_SRC_PLUGINS_ROBOT_MEMORY_EVENT_TRIGGER_H_ */
