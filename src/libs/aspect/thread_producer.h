
/***************************************************************************
 *  thread_producer.h - Thread producer aspect for Fawkes
 *
 *  Created: Tue Nov 20 11:24:35 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __ASPECT_THREAD_PRODUCER_H_
#define __ASPECT_THREAD_PRODUCER_H_

#include <core/threading/thread_collector.h>

class ThreadProducerAspect
{
 public:
  virtual ~ThreadProducerAspect();

  void init_ThreadProducerAspect(ThreadCollector *collector);

 protected:
  ThreadCollector *thread_collector;
};

#endif
