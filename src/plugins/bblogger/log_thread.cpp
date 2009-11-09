
/***************************************************************************
 *  log_thread.cpp - BB Logger Thread
 *
 *  Created: Sun Nov 08 00:02:09 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include "log_thread.h"

#include <blackboard/blackboard.h>
#include <utils/logging/logger.h>
#include <core/exceptions/system.h>

#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <cerrno>

using namespace fawkes;

/** @class BBLoggerThread "log_thread.h"
 * BlackBoard logger thread.
 * One instance of this thread handles logging of one specific interface.
 * The plugin will spawn as many threads as there are interfaces to log. This
 * allows for maximum concurrency of the writers and avoids s serialization
 * bottle neck.
 * The interface listener listens for events for a particular interface and
 * then writes the changes to the file.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param confpath configuration path to get interface name from
 * @param logdir directory to store config files, must exist
 */
BBLoggerThread::BBLoggerThread(const char *iface_uid,
			       const char *logdir)
  : Thread("BBLoggerThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("BBLoggerThread(%s)", iface_uid)
{
  __uid      = strdup(iface_uid);
  __logdir   = strdup(logdir);
  __filename = NULL;
}


/** Destructor. */
BBLoggerThread::~BBLoggerThread()
{
  free(__uid);
  free(__logdir);
  if (__filename) free(__filename);
}


void
BBLoggerThread::init()
{
  char date[18];
  struct tm *tmp = localtime(&(clock->now().get_timeval()->tv_sec));
  strftime(date, 18, "%F-%H-%M", tmp);

  if (asprintf(&__filename, "%s/%s-%s-%s.log", LOGDIR, __iface->type(),
	       __iface->id(), date) == -1) {
    throw OutOfMemoryException("Cannot generate log name");
  }
  __fd_data = open(__filename, O_WRONLY | O_CREAT | O_EXCL);
  if ( ! __fd_data) {
    free(__filename);
    throw CouldNotOpenFileException(__filename, errno, "Failed to open log");
  }

  // open interface
  std::string uid  = __uid;
  std::string::size_type cpos = uid.find("::");
  if (cpos == std::string::npos) {
    close(__fd_data);
    throw Exception("Invalid UID specified, must contain :: separator");
  }
  std::string type = uid.substr(0, cpos);
  std::string id   = uid.substr(cpos + 2);
  try {
    blackboard->open_for_reading(type.c_str(), id.c_str());
  } catch (Exception &e) {
    close(__fd_data);
    throw;
  }

  bbil_add_data_interface(__iface);
  //bbil_add_message_interface(__iface);

  blackboard->register_listener(this, BlackBoard::BBIL_FLAG_DATA);

  logger->log_info("Logging %s to %s", __iface->uid(), __filename);
}


void
BBLoggerThread::finalize()
{
  blackboard->unregister_listener(this);
  close(__fd_data);
}


void
BBLoggerThread::loop()
{
}

bool
BBLoggerThread::bb_interface_message_received(Interface *interface,
						     Message *message) throw()
{
  // just let 'em enqueue
  return true;
}


void
BBLoggerThread::bb_interface_data_changed(Interface *interface) throw()
{
  try {
    __iface->read();
    
  } catch (Exception &e) {
    logger->log_error(bbil_name(), "Exception when data changed");
    logger->log_error(bbil_name(), e);
  }
}
