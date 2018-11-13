
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

#ifndef _PLUGINS_LASER_URG_GBX_AQT_H_
#define _PLUGINS_LASER_URG_GBX_AQT_H_

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
  bool pre_init_done_;
  unsigned int number_of_values_;
#ifdef HAVE_URG_GBX_9_11
  hokuyo_aist::HokuyoLaser *laser_;
  hokuyo_aist::HokuyoData  *data_;
#else
  hokuyoaist::Sensor *laser_;
  hokuyoaist::ScanData  *data_;
#endif

  std::string  cfg_name_;
  std::string  cfg_prefix_;

  std::map<std::string, std::string> device_info_;

  std::string  cfg_device_;

  unsigned int first_ray_;
  unsigned int last_ray_;
  unsigned int front_ray_;
  unsigned int front_idx_;
  unsigned int num_rays_;
  unsigned int slit_division_;
  float        step_per_angle_;
  float        angle_per_step_;
  float        angular_range_;
};


#endif
