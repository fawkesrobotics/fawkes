
/***************************************************************************
 *  urg_aqt.h - Thread to retrieve laser data from Hokuyo URG
 *
 *  Created: Sat Nov 28 01:29:48 2009
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

#ifndef __PLUGINS_LASER_URG_AQT_H_
#define __PLUGINS_LASER_URG_AQT_H_

#include "acquisition_thread.h"

namespace qrk {
  class UrgCtrl;
}

namespace fawkes {
  class TimeWait;
}

class HokuyoUrgAcquisitionThread : public LaserAcquisitionThread
{
 public:
  HokuyoUrgAcquisitionThread();

  // from LaserAcquisitionThread
  virtual void pre_init(fawkes::Configuration *config, fawkes::Logger *logger);

  virtual void init();
  virtual void finalize();
  virtual void loop();

 private:
  bool __pre_init_done;
  unsigned int __number_of_values;
  qrk::UrgCtrl *__ctrl;

  fawkes::TimeWait *__timer;
};


#endif
