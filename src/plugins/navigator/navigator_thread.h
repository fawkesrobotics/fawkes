
/***************************************************************************
 *  navigator_thread.h - Navigator Thread
 *
 *  Generated: Thu May 31 18:36:55 2007
 *  Copyright  2007  Martin Liebenberg
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
 
#ifndef __NAVIGATOR_NAVIGATOR_THREAD_H_
#define __NAVIGATOR_NAVIGATOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <plugins/navigator/navigator.h>

class NavigatorInterface;
class MotorInterface;
class ObjectPositionInterface;

class NavigatorThread : public Thread, public BlockedTimingAspect, public LoggingAspect,
                        public BlackBoardAspect, public Navigator, public ConfigurableAspect
{
 public:
  NavigatorThread();
  virtual ~NavigatorThread();
        
  virtual void init();
  virtual void finalize();

  virtual void loop();
  virtual void once();

 private:
    
  double old_velocity_x;
  double old_velocity_y;
  double old_velocity_rotation;
  
  std::list<Interface *> *object_interface_list;
  NavigatorInterface *navigator_interface;
  MotorInterface *motor_interface;
  ObjectPositionInterface *object_interface;
  int logger_modulo_counter;
};

#endif /*NAVIGATOR_THREAD_H_*/
