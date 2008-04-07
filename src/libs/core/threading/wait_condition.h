
/***************************************************************************
 *  wait_condition.h - condition variable implementation
 *
 *  Generated: Thu Sep 14 21:34:58 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __CORE_THREADING_WAIT_CONDITION_H_
#define __CORE_THREADING_WAIT_CONDITION_H_

class WaitConditionData;
class Mutex;

class WaitCondition {

 public:
  WaitCondition();
  ~WaitCondition();

  bool wait(unsigned int timeout_sec = 0, unsigned int timeout_nanosec = 0);
  bool wait(Mutex *mutex, unsigned int timeout_sec = 0, unsigned int timeout_nanosec = 0);
  void wake_one();
  void wake_all();

 private:
  WaitConditionData *cond_data;

};




#endif
