
/***************************************************************************
 *  urg_aqt.h - Thread to retrieve laser data from Hokuyo URG
 *
 *  Created: Sat Nov 28 01:29:48 2009
 *  Copyright  2008-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_URG_AQT_H_
#define __PLUGINS_LASER_URG_AQT_H_

#include "acquisition_thread.h"

#include <string>
#include <map>

namespace qrk {
  class UrgCtrl;
}

namespace fawkes {
  class TimeWait;
}

class HokuyoUrgAcquisitionThread : public LaserAcquisitionThread
{
 public:
  HokuyoUrgAcquisitionThread(std::string &cfg_name, std::string &cfg_prefix);

  // from LaserAcquisitionThread
  virtual void pre_init(fawkes::Configuration *config, fawkes::Logger *logger);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  std::map<std::string, std::string> get_device_info(qrk::UrgCtrl *ctrl);

 private:
  bool __pre_init_done;
  unsigned int __number_of_values;
  qrk::UrgCtrl *__ctrl;
  int           __fd;

  fawkes::TimeWait *__timer;

  std::string  __cfg_name;
  std::string  __cfg_prefix;

  std::map<std::string, std::string> __device_info;

  std::string  __cfg_device;
  std::string  __cfg_serial;
  float        __cfg_time_offset;

  unsigned int __first_ray;
  unsigned int __last_ray;
  unsigned int __front_ray;
  unsigned int __slit_division;
  float        __step_per_angle;
  float        __angle_per_step;
  float        __angular_range;
  long int     __scan_msec;
};


#endif
