
/***************************************************************************
 *  calib_thread.h - Katana calibration one-time thread
 *
 *  Created: Tue Jun 09 18:34:43 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_KATANA_CALIB_THREAD_H_
#define __PLUGINS_KATANA_CALIB_THREAD_H_

#include "motion_thread.h"

class KatanaCalibrationThread : public KatanaMotionThread
{
 public:
  KatanaCalibrationThread(fawkes::RefPtr<fawkes::KatanaController> katana, fawkes::Logger *logger);

  virtual void once();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }
};


#endif
