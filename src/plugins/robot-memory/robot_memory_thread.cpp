
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
  : Thread("RobotMemoryThread", Thread::OPMODE_WAITFORWAKEUP)
{
}


/** Destructor. */
RobotMemoryThread::~RobotMemoryThread()
{
}


void
RobotMemoryThread::init()
{
	__collection = "fawkes.msglog";
  try {
    __collection = config->get_string("/plugins/mongodb/logger_collection");
  } catch (Exception &e) {}
}


void
RobotMemoryThread::finalize()
{
}


void
RobotMemoryThread::loop()
{
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
