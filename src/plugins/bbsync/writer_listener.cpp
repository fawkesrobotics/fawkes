
/***************************************************************************
 *  writer_listener.cpp - Sync Writer Interface Listener
 *
 *  Created: Fri Jun 05 16:16:22 2009
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

#include "writer_listener.h"
#include "sync_thread.h"

#include <blackboard/blackboard.h>
#include <logging/logger.h>

using namespace fawkes;

/** @class SyncWriterInterfaceListener "writer_listener.h"
 * Listener for writer events in bbsync plugin.
 * This class provides an adapter which reacts to writer events for a given
 * number of (reading) interfaces. Note that the listener is <i>not</i>
 * automatically registered, this has to be done from the outside.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param sync_thread parent sync thread to sent events to
 * @param logger logger for informational output
 * @param desc description for this interface combination
 */
SyncWriterInterfaceListener::SyncWriterInterfaceListener(BlackBoardSynchronizationThread *sync_thread,
							 fawkes::Logger *logger,
							 const char *desc)
  : BlackBoardInterfaceListener("SyncWriterInterfaceListener(%s)", desc)
{
  __logger      = logger;
  __sync_thread = sync_thread;
}


/** Add an interface to listen to.
 * @param interface interface to listen to for writer events
 */
void
SyncWriterInterfaceListener::add_interface(Interface *interface)
{
  bbil_add_writer_interface(interface);
}


/** Remove an interface to listen to.
 * @param interface interface not to listen any longer for writer events
 */
void
SyncWriterInterfaceListener::remove_interface(Interface *interface)
{
  bbil_remove_writer_interface(interface);
}

void
SyncWriterInterfaceListener::bb_interface_writer_added(fawkes::Interface *interface,
						       unsigned int instance_serial) throw()
{
  __sync_thread->writer_added(interface);
}


void
SyncWriterInterfaceListener::bb_interface_writer_removed(fawkes::Interface *interface,
							 unsigned int instance_serial) throw()
{
  __sync_thread->writer_removed(interface);
}
