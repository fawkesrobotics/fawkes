
/***************************************************************************
 *  semset.h - IPC semaphore set
 *
 *  Generated: Tue Sep 19 14:45:47 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_IPC_SEMSET_H_
#define __UTILS_IPC_SEMSET_H_

namespace fawkes {


class SemaphoreSetData;

class SemaphoreSet {
 public:

  SemaphoreSet(const char *path, char id,
	       int num_sems,
	       bool create = false,
	       bool destroy_on_delete = false);

  SemaphoreSet(int key,
	       int num_sems,
	       bool create = false,
	       bool destroy_on_delete = false);

  SemaphoreSet( int num_sems,
	        bool destroy_on_delete = false);

  ~SemaphoreSet();

  bool valid();
  void lock(unsigned short sem_num = 0, short num = 1);
  bool try_lock(unsigned short sem_num = 0, short num = 1);
  void unlock(unsigned short sem_num = 0, short num = -1);
  void set_value(int sem_num, int val);
  int  get_value(int sem_num);
  int  key();
  void set_destroy_on_delete(bool destroy);

  static int  free_key();
  static void destroy(int key);

 protected:
  bool destroy_on_delete;

 private:
  SemaphoreSetData *data;

};


} // end namespace fawkes

#endif
