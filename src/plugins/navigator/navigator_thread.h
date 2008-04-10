
/***************************************************************************
 *  navigator_thread.h - Navigator Thread
 *
 *  Generated: Thu May 31 18:36:55 2007
 *  Copyright  2007  Martin Liebenberg
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
 
#ifndef __NAVIGATOR_NAVIGATOR_THREAD_H_
#define __NAVIGATOR_NAVIGATOR_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <blackboard/interface_listener.h>
#include <blackboard/interface_observer.h>
#include <plugins/navigator/navigator.h>

class NavigatorInterface;
class MotorInterface;
class ObjectPositionInterface;
class Mutex;

class NavigatorThread
: public Thread,
  public BlockedTimingAspect,
  public LoggingAspect,
  public BlackBoardAspect,
  public ConfigurableAspect,
  public BlackBoardInterfaceListener,
  public BlackBoardInterfaceObserver,
  public Navigator
{
 public:
  NavigatorThread();
  virtual ~NavigatorThread();
        
  virtual void init();
  virtual void finalize();

  virtual void loop();
  virtual void once();
  
  virtual void bb_interface_created(const char *type, const char *id) throw();
  
  double get_ball_position_x();
  double get_ball_position_y();

 private:
    
  double old_velocity_x;
  double old_velocity_y;
  double old_velocity_rotation;
  
  double ball_position_x;
  double ball_position_y;
  
  Mutex *ball_mutex;
  
  std::list<ObjectPositionInterface *> *object_interface_list;
  std::list<ObjectPositionInterface *>::iterator oili;
  NavigatorInterface *navigator_interface;
  MotorInterface *motor_interface;
  ObjectPositionInterface *object_interface;
  int logger_modulo_counter;
};

#endif /*NAVIGATOR_THREAD_H_*/
