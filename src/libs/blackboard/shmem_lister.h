
/***************************************************************************
 *  shmem_lister.h - BlackBoard shared memory lister
 *
 *  Created: Fri Oct 20 11:40:09 2006
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

#ifndef __BLACKBOARD_SHMEM_LISTER_H_
#define __BLACKBOARD_SHMEM_LISTER_H_

#include <utils/ipc/shm_lister.h>

class SharedMemoryHeader;

class BlackBoardSharedMemoryLister : public SharedMemoryLister {
 public:

  BlackBoardSharedMemoryLister();
  virtual ~BlackBoardSharedMemoryLister();

  virtual void printHeader();
  virtual void printFooter();
  virtual void printNoSegments();
  virtual void printNoOrphanedSegments();

  virtual void printInfo(SharedMemoryHeader *header,
			 int shm_id, int semaphore,
			 unsigned int mem_size, void *memptr);

 private:
  unsigned int num;
};

#endif
