
/***************************************************************************
 *  finalize_nettler_thread.h - Fawkes Example Plugin Finalize Nettler Thread
 *
 *  Created: Thu May 24 00:32:22 2007
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_EXAMPLE_FINALIZE_NETTLER_THREAD_H_
#define __PLUGINS_EXAMPLE_FINALIZE_NETTLER_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>

class ExampleFinalizeNettlerThread
: public fawkes::Thread,
  public fawkes::LoggingAspect
{

 public:
  ExampleFinalizeNettlerThread(const char *name);
  virtual ~ExampleFinalizeNettlerThread();

  virtual void init();
  virtual void loop();

  virtual bool prepare_finalize_user();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  bool nagged;
};


#endif
