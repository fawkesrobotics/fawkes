
/***************************************************************************
 *  goto_thread.h - Katana goto one-time thread
 *
 *  Created: Wed Jun 10 11:44:24 2009
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

#ifndef __PLUGINS_KATANA_GOTO_THREAD_H_
#define __PLUGINS_KATANA_GOTO_THREAD_H_

#include "motion_thread.h"

#include <string>

class KatanaGotoThread : public KatanaMotionThread
{
 public:
  KatanaGotoThread(fawkes::RefPtr<fawkes::KatanaController> katana, fawkes::Logger *logger,
		   unsigned int poll_interval_ms);

  virtual void set_target(float x, float y, float z, float phi, float theta, float psi);

  virtual void once();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 private:
  virtual void run() { Thread::run(); }

  float __x, __y, __z;
  float __phi, __theta, __psi;
  unsigned int __poll_interval_usec;
};


#endif
