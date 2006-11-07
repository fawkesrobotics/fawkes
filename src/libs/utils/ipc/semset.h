
/***************************************************************************
 *  semset.h - IPC semaphore set
 *
 *  Generated: Tue Sep 19 14:45:47 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_IPC_SEMSET_H_
#define __UTILS_IPC_SEMSET_H_

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

  bool isValid();
  void lock(unsigned short sem_num = 0, short num = 1);
  bool tryLock(unsigned short sem_num = 0, short num = 1);
  void unlock(unsigned short sem_num = 0, short num = -1);
  void setVal(int sem_num, int val);
  int  getVal(int sem_num);
  int  getKey();
  void setDestroyOnDelete(bool destroy);

  static int getFreeKey();
  static void destroy(int key);

 protected:
  bool destroy_on_delete;

 private:
  SemaphoreSetData *data;

};

#endif
