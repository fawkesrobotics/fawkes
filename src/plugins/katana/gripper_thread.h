
/***************************************************************************
 *  gripper_thread.h - Katana gripper one-time thread
 *
 *  Created: Thu Jun 11 11:56:46 2009
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

#ifndef __PLUGINS_KATANA_GRIPPER_THREAD_H_
#define __PLUGINS_KATANA_GRIPPER_THREAD_H_

#include "motion_thread.h"

class KatanaGripperThread : public KatanaMotionThread
{
 public:
  KatanaGripperThread(fawkes::RefPtr<fawkes::KatanaController> katana, fawkes::Logger *logger,
		      unsigned int poll_interval_ms);

  /** Gripper execution mode. */
  typedef enum {
    OPEN_GRIPPER,	/**< Open gripper */
    CLOSE_GRIPPER	/**< Close gripper */
  } gripper_mode_t;

  void set_mode(gripper_mode_t mode);
  virtual void once();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  gripper_mode_t  __mode;
  unsigned int    __poll_interval_usec;
};


#endif
