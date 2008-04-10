
/***************************************************************************
 *  kicker_thread.h - Fawkes Kicker Plugin Thread
 *
 *  Generated: Fri May 11 16:06:46 2007
 *  Copyright  2007  Daniel Beck
 *
 *  $Id$
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

#ifndef __PLUGINS_KICKER_THREAD_H_
#define __PLUGINS_KICKER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>

class KickerControl;
class KickerInterface;

class KickerThread
  : public Thread,
    public BlockedTimingAspect,
    public LoggingAspect,
    public BlackBoardAspect
{
 public:
  KickerThread();
  virtual ~KickerThread();

  virtual void finalize();
  virtual void init();
  virtual void loop();

 private:
  KickerInterface* kicker_interface;  
  KickerControl* kicker_control;

};

#endif /* __PLUGINS_KICKER_THREAD_H_ */
                  
