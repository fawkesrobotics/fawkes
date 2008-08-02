
/***************************************************************************
 *  mainloop.h - Main loop aspect for Fawkes
 *
 *  Created: Sat Aug  2 00:10:32 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#ifndef __ASPECT_MAINLOOP_H_
#define __ASPECT_MAINLOOP_H_

#include <aspect/mainloop/mainloop.h>
#include <aspect/blocked_timing/executor.h>

namespace fawkes {

class BlockedTimingExecutor;

class MainLoopAspect
{
 public:
  MainLoopAspect(MainLoop *mainloop) __attribute__((nonnull));
  virtual ~MainLoopAspect();

  void init_MainLoopAspect(BlockedTimingExecutor *btexec);
  MainLoop *  get_mainloop() const;

 protected:
  BlockedTimingExecutor *blocked_timing_executor;

 private:
  MainLoop *__mainloop;
};

} // end namespace fawkes

#endif
