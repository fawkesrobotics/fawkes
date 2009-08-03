
/***************************************************************************
 *  writer_listener.h - Sync Writer Interface Listener
 *
 *  Created: Fri Jun 05 16:14:37 2009
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

#ifndef __PLUGINS_BBSYNC_WRITER_LISTENER_H_
#define __PLUGINS_BBSYNC_WRITER_LISTENER_H_

#include <blackboard/interface_listener.h>

namespace fawkes {
  class BlackBoard;
  class Logger;
}

class BlackBoardSynchronizationThread;

class SyncWriterInterfaceListener
: public fawkes::BlackBoardInterfaceListener
{
 public:
  SyncWriterInterfaceListener(BlackBoardSynchronizationThread *sync_thread,
			      fawkes::Logger *logger, const char *desc);

  void add_interface(fawkes::Interface *interface);
  void remove_interface(fawkes::Interface *interface);

  // BlackBoardInterfaceListener
  virtual void bb_interface_writer_added(fawkes::Interface *interface,
					 unsigned int instance_serial) throw();
  virtual void bb_interface_writer_removed(fawkes::Interface *interface,
					   unsigned int instance_serial) throw();

 private:
  fawkes::Logger                   *__logger;
  BlackBoardSynchronizationThread  *__sync_thread;
};


#endif
