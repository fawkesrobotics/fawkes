
/***************************************************************************
 *  exec_thread.h - Fawkes Skiller: Execution Thread
 *
 *  Created: Mon Feb 18 10:28:38 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __PLUGINS_SKILLER_EXEC_THREAD_H_
#define __PLUGINS_SKILLER_EXEC_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>

extern "C" {
#include <lua.h>
}

#include <string>

class SkillerLiaisonThread;
class ComponentLogger;

class SkillerExecutionThread
: public Thread,
  public BlockedTimingAspect,
  public LoggingAspect,
  public ConfigurableAspect,
  public ClockAspect
{
 public:
  SkillerExecutionThread(Barrier *liaison_exec_barrier, SkillerLiaisonThread *slt);
  virtual ~SkillerExecutionThread();

  virtual void init();
  virtual void once();
  virtual void loop();
  virtual void finalize();

 private:
  Barrier *__liaison_exec_barrier;
  SkillerLiaisonThread *__slt;
  ComponentLogger *__clog;

  lua_State *L;
  int err;
  std::string errmsg;
};

#endif
