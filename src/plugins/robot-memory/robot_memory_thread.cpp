
/***************************************************************************
 *  robot_memory_thread.cpp - Robot Memory thread
 *
 *  Created: Sun May 01 13:41:45 2016
 *  Copyright  2016 Frederik Zwilling
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

#include "robot_memory_thread.h"
#include "interfaces/RobotMemoryInterface.h"
#include <core/threading/mutex.h>
#include <core/threading/mutex_locker.h>
#include <memory>

// from MongoDB
#include <mongo/client/dbclient.h>

using namespace mongo;
using namespace fawkes;

/** @class RobotMemoryThread "robot_memory_thread.h"
 * Thread that provides a robot memory with MongoDB
 * @author Frederik Zwilling
 */

/** Constructor. */
RobotMemoryThread::RobotMemoryThread()
	: Thread("RobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP),
	  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_WORLDSTATE)
{
  __mutex = new Mutex();
}


/** Destructor. */
RobotMemoryThread::~RobotMemoryThread()
{
  delete __mutex;
}


void
RobotMemoryThread::init()
{
	logger->log_info(name(), "Started RobotMemory");
	__collection = "fawkes.msglog";
  try {
    __collection = config->get_string("/plugins/mongodb/logger_collection");
  } catch (Exception &e) {}

  __rm_if = blackboard->open_for_writing<RobotMemoryInterface>(config->get_string("/plugins/robot-memory/interface-name").c_str());
  __rm_if->set_error("");
  __rm_if->set_result("");
  
  __rm_if->write();
}


void
RobotMemoryThread::finalize()
{
}


void
RobotMemoryThread::loop()
{
	// process interface messages
  while (! __rm_if->msgq_empty() ) {
    if (__rm_if->msgq_first_is<RobotMemoryInterface::QueryMessage>()) {
	    RobotMemoryInterface::QueryMessage* query = (RobotMemoryInterface::QueryMessage*) __rm_if->msgq_first();
	    exec_query(query->query());
    } else {
      logger->log_warn(name(), "Unknown message received");
    }

    __rm_if->msgq_pop();
  }
}

void RobotMemoryThread::exec_query(std::string query_string)
{
	logger->log_info(name(), "Executing Query: %s", query_string.c_str());

	//only one query at a time
	MutexLocker lock(__mutex);

	//get query from string
	Query query;
	try{
	  query = Query(query_string);
	} catch (DBException &e) {
		logger->log_error(name(), "Can't parse query_string '%s'\n Exception: %s",
		                  query_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Can't parse query_string ") +  query_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	//introspect query
	logger->log_info(name(), "Filter: %s", query.getFilter().toString().c_str());
	logger->log_info(name(), "Modifiers: %s", query.getModifiers().toString().c_str());
	logger->log_info(name(), "Sort: %s", query.getSort().toString().c_str());
	logger->log_info(name(), "Hint: %s", query.getHint().toString().c_str());
	logger->log_info(name(), "ReadPref: %s", query.getReadPref().toString().c_str());

	
	//actually execute query
	std::unique_ptr<DBClientCursor> cursor;
	try{
	  cursor = mongodb_client->query(__collection, query);
	} catch (DBException &e) {
		logger->log_error(name(), "Error for query %s\n Exception: %s",
		                  query_string.c_str(), e.toString().c_str());
		__rm_if->set_error((std::string("Query error for ") +  query_string
		                    + "\nException: " + e.toString()).c_str());
		__rm_if->write();
		return;
	}

	logger->log_info(name(), "Query One result:\n%s", cursor->next().toString().c_str());
	__rm_if->set_result(cursor->next().toString().c_str());
	__rm_if->write();
}

// void
// RobotMemoryThread::insert_message(LogLevel ll, const char *component,
// 				    const char *format, va_list va)
// {
//   if (log_level <= ll ) {
//     MutexLocker lock(__mutex);
//     struct timeval now;
//     gettimeofday(&now, NULL);
//     Date_t nowd = now.tv_sec * 1000 + now.tv_usec / 1000;

//     char *msg;
//     if (vasprintf(&msg, format, va) == -1) {
//       // Cannot do anything useful, drop log message
//       return;
//     }

//     BSONObjBuilder b;
//     switch (ll) {
//     case LL_DEBUG: b.append("level", "DEBUG"); break;
//     case LL_INFO:  b.append("level", "INFO");  break;
//     case LL_WARN:  b.append("level", "WARN");  break;
//     case LL_ERROR: b.append("level", "ERROR"); break;
//     default:       b.append("level", "UNKN");  break;
//     }
//     b.append("component", component);
//     b.appendDate("time", nowd);
//     b.append("message", msg);

//     free(msg);

//     try {
//       mongodb_client->insert(__collection, b.obj());
//     } catch (mongo::DBException &e) {} // ignored
//   }
// }
