
/***************************************************************************
 *  wm_thread.h - Fawkes WorldModel Plugin Thread
 *
 *  Created: Fri Jun 29 11:54:58 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WORLDMODEL_WM_THREAD_H_
#define __PLUGINS_WORLDMODEL_WM_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>

#include <list>

class ObjectPositionInterface;

class WorldModelThread
: public Thread,
  public BlockedTimingAspect,
  public LoggingAspect,
  public ConfigurableAspect,
  public BlackBoardAspect,
  public ClockAspect,
  public BlackBoardInterfaceListener,
  public BlackBoardInterfaceObserver
{
 public:
  WorldModelThread();
  virtual ~WorldModelThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  void init_failure_cleanup();

 private:
  ObjectPositionInterface *wm_ball_interface;
  ObjectPositionInterface *wm_pose_interface;
  std::list<ObjectPositionInterface *>  *wm_opp_interfaces;

  std::list<ObjectPositionInterface *>  *in_opp_interfaces;
  std::list<ObjectPositionInterface *>  *in_ball_interfaces;

  std::list<ObjectPositionInterface *>::iterator opii;
};


#endif
