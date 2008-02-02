
/***************************************************************************
 *  thread.h - Fawkes Example Plugin Thread
 *
 *  Generated: Wed Nov 22 17:06:33 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __PLUGINS_EXAMPLE_THREAD_H_
#define __PLUGINS_EXAMPLE_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>

class ExampleThread : public Thread, public BlockedTimingAspect, public LoggingAspect
{

 public:
  ExampleThread(BlockedTimingAspect::WakeupHook hook, const char *name, unsigned int modc);
  virtual ~ExampleThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  unsigned int m;
  unsigned int modc;
};


#endif
