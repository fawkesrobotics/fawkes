
/***************************************************************************
 *  sync_thread.h - Fawkes BlackBoard Synchronization Thread
 *
 *  Created: Thu Jun 04 18:10:17 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __PLUGINS_BBSYNC_SYNC_THREAD_H_
#define __PLUGINS_BBSYNC_SYNC_THREAD_H_

#include "sync_listener.h"
#include "writer_listener.h"

#include <core/threading/thread.h>
#include <core/utils/lock_map.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>

#include <string>
#include <map>
#include <utility>

namespace fawkes {
  class TimeWait;
}

class BlackBoardSynchronizationThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ClockAspect
{
 public:
  BlackBoardSynchronizationThread(std::string &bbsync_cfg_prefix,
				  std::string &peer_cfg_prefix, std::string &peer);
  virtual ~BlackBoardSynchronizationThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  void writer_added(fawkes::Interface *interface) throw();
  void writer_removed(fawkes::Interface *interface) throw();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  typedef struct {
    std::string type;
    std::string reader_id;
    std::string writer_id;
    bool remote_writer;
  } combo_t;

  class InterfaceInfo {
   public:
    combo_t            *combo;
    fawkes::Interface  *writer;
    fawkes::BlackBoard *reader_bb;
    fawkes::BlackBoard *writer_bb;

    InterfaceInfo()
      : combo(NULL), writer(NULL), reader_bb(NULL), writer_bb(NULL)
    {}

    InterfaceInfo(combo_t *pcombo, fawkes::Interface  *pwriter,
		  fawkes::BlackBoard *preader_bb, fawkes::BlackBoard *pwriter_bb)
      : combo(pcombo), writer(pwriter), reader_bb(preader_bb), writer_bb(pwriter_bb)
    {}

    InterfaceInfo & operator=(const InterfaceInfo &ii)
    {
      combo=ii.combo; writer=ii.writer; reader_bb=ii.reader_bb; writer_bb=ii.writer_bb;
      return *this;
    }
  };

  typedef std::map<std::string, combo_t > ComboMap;
  typedef fawkes::LockMap<fawkes::Interface *, InterfaceInfo> InterfaceMap;
  typedef fawkes::LockMap<fawkes::Interface *, SyncInterfaceListener *> SyncListenerMap;

  bool check_connection();
  void read_config_combos(std::string prefix, bool writing);
  void open_interfaces();
  void close_interfaces();

 private:
  std::string   __bbsync_cfg_prefix;
  std::string   __peer_cfg_prefix;
  std::string   __peer;

  std::string   __host;
  unsigned int  __port;

  fawkes::TimeWait    *__timewait;

  fawkes::BlackBoard  *__remote_bb;

  ComboMap __combos;

  // Maps reading -> writing interface
  InterfaceMap __interfaces;
  // Maps reading interface -> sync lsitener
  SyncListenerMap __sync_listeners;

  SyncWriterInterfaceListener *__wsl_local;
  SyncWriterInterfaceListener *__wsl_remote;
};


#endif
