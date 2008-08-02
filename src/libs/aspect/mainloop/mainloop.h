
/***************************************************************************
 *  mainloop.h - Fawkes main loop interface
 *
 *  Created: Sat Aug  2 00:03:17 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_MAINLOOP_MAINLOOP_H_
#define __ASPECT_MAINLOOP_MAINLOOP_H_


namespace fawkes {

class BlockedTimingExecutor;

class MainLoop
{
 public:
  virtual ~MainLoop();

  virtual void mloop() = 0;
  void set_blocked_timing_executor(BlockedTimingExecutor *btexec);

 protected:
  BlockedTimingExecutor *_btexec;
};

} // end of namespace fawkes

#endif
