
/***************************************************************************
 *  sensor_com_handler.h - Robotino sensor Com handler
 *
 *  Created: Sun Jul 06 19:49:47 2014
 *  Copyright  2011-2014  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROBOTINO_SENSOR_COM_HANDLER_H_
#define __PLUGINS_ROBOTINO_SENSOR_COM_HANDLER_H_

#include <rec/robotino/com/Com.h>
#include <rec/iocontrol/remotestate/SensorState.h>
#include <utils/time/time.h>

namespace fawkes {
  class Mutex;
  class Clock;
}

class RobotinoSensorComHandler : public rec::robotino::com::Com
{
 public:
  RobotinoSensorComHandler(fawkes::Clock *clock);
  virtual ~RobotinoSensorComHandler();

  virtual void updateEvent();

  rec::iocontrol::remotestate::SensorState
    sensor_state(fawkes::Time &time) const;

 private:
  using rec::robotino::com::Com::sensorState;

 private:
  fawkes::Mutex *mutex_;
  unsigned int active_state_;
  rec::iocontrol::remotestate::SensorState sensor_states_[2];
  fawkes::Time times_[2];
};


#endif
