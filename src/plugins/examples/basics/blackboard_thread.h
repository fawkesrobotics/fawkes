
/***************************************************************************
 *  blackboard_thread.h - Fawkes Example Plugin BlackBoard Thread
 *
 *  Created: Wed Jun 20 16:35:47 2007
 *  Copyright  2007-2008  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_EXAMPLE_BLACKBOARD_THREAD_H_
#define __PLUGINS_EXAMPLE_BLACKBOARD_THRED_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>

class TestInterface;

class ExampleBlackBoardThread
  : public Thread,
    public BlockedTimingAspect,
    public LoggingAspect,
    public BlackBoardAspect
{
 public:
  ExampleBlackBoardThread(bool reader);
  virtual ~ExampleBlackBoardThread();

  virtual void finalize();
  virtual void init();
  virtual void loop();

 private:
  TestInterface* test_interface;  
  bool           reader;
};

#endif
