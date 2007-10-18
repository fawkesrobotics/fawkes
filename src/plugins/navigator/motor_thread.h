
/***************************************************************************
 *  motor_thread.h - Motor Thread
 *
 *  Generated: Son Jun 03 00:07:33 2007
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
 
#ifndef __NAVIGATOR_MOTOR_THREAD_H_
#define __NAVIGATOR_MOTOR_THREAD_H_

#include <core/threading/thread.h>
#include <core/threading/thread_notification_listener.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>

class MotorInterface;
class NavigatorThread;

namespace VMC 
{
  class CvmcAPI;
}


class MotorThread : public Thread, public LoggingAspect, public BlackBoardAspect, 
                    public ConfigurableAspect
{
 public:
  MotorThread();
        
  virtual ~MotorThread();
        
  virtual void loop();
  virtual void init();
  virtual void finalize();
    
 private:
  VMC::CvmcAPI *apiObject;
    
  MotorInterface *motor_interface;
        
  bool extern_control;
        
  double forward;
  double sideward;
  double rotation;
  double speed;

  double old_alpha;
  double old_beta;
  double old_gamma;
  
  int timeout_counter;  
    
  unsigned int logger_modulo_counter;
};

#endif /*MOTOR_THREAD_H_*/
