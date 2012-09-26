
/***************************************************************************
 *  urg_gbx_aqt.h - Thread for Hokuyo URG using the Gearbox library
 *
 *  Created: Fri Dec 04 20:30:08 2009 (at Frankfurt Airport)
 *  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_LASER_URG_GBX_AQT_H_
#define __PLUGINS_LASER_URG_GBX_AQT_H_

#include "acquisition_thread.h"

#include <string>
#include <map>

#ifdef HAVE_URG_GBX_9_11
namespace hokuyo_aist {
  class HokuyoLaser;
  class HokuyoData;
}
#else
namespace hokuyoaist {
  class Sensor;
  class ScanData;
}
#endif

class HokuyoUrgGbxAcquisitionThread : public LaserAcquisitionThread
{
 public:
  HokuyoUrgGbxAcquisitionThread(std::string &cfg_name, std::string &cfg_prefix);

  // from LaserAcquisitionThread
  virtual void pre_init(fawkes::Configuration *config, fawkes::Logger *logger);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  bool __pre_init_done;
  unsigned int __number_of_values;
#ifdef HAVE_URG_GBX_9_11
  hokuyo_aist::HokuyoLaser *__laser;
  hokuyo_aist::HokuyoData  *__data;
#else
  hokuyoaist::Sensor *__laser;
  hokuyoaist::ScanData  *__data;
#endif

  std::string  __cfg_name;
  std::string  __cfg_prefix;

  std::map<std::string, std::string> __device_info;

  std::string  __cfg_device;

  unsigned int __first_ray;
  unsigned int __last_ray;
  unsigned int __front_ray;
  unsigned int __front_idx;
  unsigned int __num_rays;
  unsigned int __slit_division;
  float        __step_per_angle;
  float        __angle_per_step;
  float        __angular_range;
};


#endif
