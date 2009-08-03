
/***************************************************************************
 *  shmem_lister.h - BlackBoard shared memory lister
 *
 *  Created: Fri Oct 20 11:40:09 2006
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

#ifndef __BLACKBOARD_SHMEM_LISTER_H_
#define __BLACKBOARD_SHMEM_LISTER_H_

#include <utils/ipc/shm_lister.h>

namespace fawkes {

class SharedMemoryHeader;

class BlackBoardSharedMemoryLister : public SharedMemoryLister {
 public:

  BlackBoardSharedMemoryLister();
  virtual ~BlackBoardSharedMemoryLister();

  virtual void print_header();
  virtual void print_footer();
  virtual void print_no_segments();
  virtual void print_no_orphaned_segments();

  virtual void print_info(const SharedMemoryHeader *header,
			  int shm_id, int semaphore,
			  unsigned int mem_size, const void *memptr);

 private:
  unsigned int num;
};

} // end namespace fawkes

#endif
