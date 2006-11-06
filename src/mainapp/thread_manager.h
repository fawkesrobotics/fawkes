
/***************************************************************************
 *  thread_manager.h - Fawkes thread manager
 *
 *  Created: Thu Nov  3 19:08:23 2006 (on train to Cologne)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __FAWKES_THREAD_MANAGER_H_
#define __FAWKES_THREAD_MANAGER_H_

#include <core/threading/thread_list.h>
#include <core/threading/fawkes_thread.h>
#include <core/exception.h>

#include <map>
#include <string>

class BlackBoardInterfaceManager;
class PluginLoader;
class Plugin;
class Barrier;

class NoInterfaceManagerException : public Exception
{
 public:
  NoInterfaceManagerException()
    : Exception("No BlackBoard main thread and thus no InterfaceManager known") {};
};


class InvalidWakeupHookException : public Exception
{
 public:
  InvalidWakeupHookException()
    : Exception("The WAKEUP_HOOK_NONE wakeup hook cannot be woken up!") {};
};


class FawkesThreadManager
{
 public:
  FawkesThreadManager();
  ~FawkesThreadManager();

  void add(ThreadList &tl);
  void remove(ThreadList &tl);
  void wakeup(FawkesThread::WakeupHook hook);
  void wait(FawkesThread::WakeupHook hook);

  BlackBoardInterfaceManager *  getInterfaceManager() const;

 private:
  void start(ThreadList &tl);
  void stop(ThreadList &tl);

  BlackBoardInterfaceManager * interface_manager;

  std::map< FawkesThread::WakeupHook, ThreadList > threads;
  std::map< FawkesThread::WakeupHook, ThreadList >::iterator tit;

  std::map< FawkesThread::WakeupHook, Barrier * >  barriers;

};

#endif
