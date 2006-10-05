
/***************************************************************************
 *  shm_lister.h - shared memory buffer lister
 *
 *  Generated: Sun Sep 17 14:23:34 2006 (split from shm.h)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_IPC_SHM_LISTER_H_
#define __UTILS_IPC_SHM_LISTER_H_


/** Format list output for shared memory segments.
 * Implement this function specific to your SharedMemoryHeader to printout
 * data about the shared memory segments.
 * @ingroup IPC
 * @author Tim Niemueller
 */
class SharedMemoryLister {
 public:

  /** virtual destructor */
  virtual ~SharedMemoryLister() {}

  /** Print header of the table.
   * This should fit on the terminal and thus have a width of at most
   * 79 columns.
   */
  virtual void printHeader()                                 = 0;

  /** Print footer of the table.
   * This should fit on the terminal and thus have a width of at most
   * 79 columns.
   */
  virtual void printFooter()                                 = 0;

  /** Print this if no matching segment was found.
   * Called by SharedMemory if no matching segment could be found.
   */
  virtual void printNoSegments()                             = 0;

  /** Print info about segment.
   * This method is called for every matching shared memory segment.
   * You should print a line of information (maybe more than one line
   * if needed) about the segment.
   * @param header The data-specific header
   * @param shm_id The id of the shared memory segment
   * @param mem_size the total memory size
   * @param memptr pointer to the data segment.
   */
  virtual void printInfo(SharedMemoryHeader *header,
			 int shm_id, unsigned int mem_size,
			 void *memptr)                = 0;
};


#endif
