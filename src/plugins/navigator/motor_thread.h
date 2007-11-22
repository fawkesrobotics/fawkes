
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
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <utils/time/time.h>

class MotorInterface;
class NavigatorThread;
class Clock;
class Watch;

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

    bool no_vmc;

    double forward;
    double sideward;
    double rotation;
    double orbit_velocity;
    double point_x;
    double point_y;
    double orbit_angular_velocity;
    double alpha;
    double beta;
    double gamma;
    double alpha_;
    double beta_;
    double gamma_;
    double odometry_distance;
    double orbit_direction_x;
    double orbit_direction_y;
    // double orbit_rotation_velocity;
    double last_velocity;
    double current_velocity;
    double current_max_velocity;

    double old_alpha;
    double old_beta;
    double old_gamma;

    double correction_x;
    double correction_y;
    double correction_rotation;
    double  correction_translation;
    double  gear_reduction;
    double  wheel_radius;
    double  radius;
    double  differential_part;
    double  integral_part;
    double  linear_part;
    int  ticks;
    
    double translation_rpm_factor;
    double rotation_rpm_factor;

    Time last_time;
    Time last_time_odometry;
    Watch* stop_watch;
    Time* stop_time;
    Time last_acceleration_time;

    Clock* clock;

    unsigned int logger_modulo_counter;
  };

#endif /*MOTOR_THREAD_H_*/
