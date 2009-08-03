
/***************************************************************************
 *  shmem_lister.cpp - BlackBoard shared memory lister
 *
 *  Created: Fri Oct 20 11:50:03 2006
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

#include <blackboard/shmem/lister.h>
#include <utils/system/console_colors.h>
#include <utils/ipc/shm.h>

#include <iostream>
#include <cstdio>

using namespace std;
namespace fawkes {

/** @class BlackBoardSharedMemoryLister <blackboard/shmem/lister.h>
 * BlackBoard shared memory lister.
 * Lister that can be used to print infos about BlackBoard shared memory
 * segments.
 * @author Tim Niemueller
 */

/** Constructor */
BlackBoardSharedMemoryLister::BlackBoardSharedMemoryLister()
{
  num = 0;
}


/** Destructor */
BlackBoardSharedMemoryLister::~BlackBoardSharedMemoryLister()
{
}


/** Print header of the table.
 * This should fit on the terminal and thus have a width of at most
 * 79 columns.
 */
void
BlackBoardSharedMemoryLister::print_header()
{
  cout << endl << cblue << "Fawkes BlackBoard Shared Memory Segments" << cnormal << endl
       << "========================================================================" << endl
       << cdarkgray;
  printf ("%-3s %-10s %-11s %-16s %-12s %s\n",
          "#", "ShmID", "Semaphore", "Bytes", "# attached", "State");
  cout << cnormal
       << "------------------------------------------------------------------------" << endl;
  num = 0;
}


/** Print footer of the table.
 * This should fit on the terminal and thus have a width of at most
 * 79 columns.
 */
void
BlackBoardSharedMemoryLister::print_footer()
{
  cout << "========================================================================" << endl;
}


/** Print this if no matching segment was found.
 * Called by SharedMemory if no matching segment could be found.
 */
void
BlackBoardSharedMemoryLister::print_no_segments()
{
  cout << "No Fawkes BlackBoard shared memory segments found" << endl;
}


/** Print this if no matching orphaned segment was found.
 * Called by SharedMemory::erase_orphaned() if no matching segment
 * could be found.
 */
void
BlackBoardSharedMemoryLister::print_no_orphaned_segments()
{
  cout << "No " << cdarkgray << "orphaned" << cnormal
       << " Fawkes BlackBoard shared memory segments found" << endl;
}


/** Print info about segment.
 * This method is called for every matching shared memory segment.
 * You should print a line of information (maybe more than one line
 * if needed) about the segment.
 * @param header The data-specific header
 * @param shm_id The id of the shared memory segment
 * @param semaphore semaphore assigned to the shared memory segment
 * @param mem_size the total memory size
 * @param memptr pointer to the data segment.
 */
void
BlackBoardSharedMemoryLister::print_info(const SharedMemoryHeader *header,
					 int shm_id, int semaphore,
					 unsigned int mem_size,
					 const void *memptr)
{
  unsigned int nattch = SharedMemory::num_attached(shm_id);
  bool swapable = SharedMemory::is_swapable(shm_id);
  bool destroyed = SharedMemory::is_destroyed(shm_id);

  printf ("%-3u %-10d 0x%08x  %-16u %-12u %s%s%s%s%s\n",
          ++num, shm_id, semaphore, mem_size, nattch,
 	  ((nattch > 1) ? "active" : "orphaned"),
	  ((swapable || destroyed) ? " (" : ""),
	  (swapable ? "S" : ""),
	  (destroyed ? "D" : ""),
	  ((swapable || destroyed) ? ")" : "")
	  );
}

} // end namespace fawkes
