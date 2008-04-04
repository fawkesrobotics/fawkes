
/***************************************************************************
 *  thread_inifin.h - Fawkes thread initializer/finalizer
 *
 *  Created: Thu Nov 20 00:47:12 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FAWKES_THREAD_INIFIN_H_
#define __FAWKES_THREAD_INIFIN_H_

#include <aspect/inifin.h>

class BlackBoard;
class Configuration;
class Logger;
class Clock;
class Thread;
class ThreadCollector;

class FawkesThreadIniFin : public AspectIniFin
{
 public:
  FawkesThreadIniFin(BlackBoard *blackboard, ThreadCollector *collector,
		     Configuration *config, Logger *logger, Clock *clock);

  virtual void init(Thread *thread);
  virtual void finalize(Thread *thread);
};


#endif
