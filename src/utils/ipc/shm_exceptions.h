
/***************************************************************************
 *  shm_exceptions.h - exceptions thrown in shmem utils, do NOT put your own
 *                     application specific exceptions here!
 *
 *  Generated: Thu Feb 09 13:06:52 2006
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_IPC_SHM_EXCEPTIONS_H_
#define __UTILS_IPC_SHM_EXCEPTIONS_H_

#include <core/exception.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>

/** Could not attach to shared memory segment.
 */
class ShmCouldNotAttachException : public Exception {
 public:
  /** Constructor
   * @param msg Message why we could not attach
   */
  ShmCouldNotAttachException(const char *msg) : Exception(msg)  {}
};


/** No shared memory header set before attach()
 */
class ShmNoHeaderException : public Exception {
 public:
  /** Constructor */
  ShmNoHeaderException() : Exception("No SharedMemoryHeader, cannot attach")  {}
};


/** Memory size does not match
 */
class ShmInconsistentSegmentSizeException : public Exception {
 public:
  /** Constructor
   * @param desired_mem The exepcted memory size
   * @param act_mem The actual memory size
   */
  ShmInconsistentSegmentSizeException(unsigned int desired_mem, unsigned int act_mem)
    : Exception()
  {
    char *message;
    asprintf( &message, "Inconsistent shared mem segment found in memory "
	               "(memory size does not match, desired: %u, actual: %u)",
	               desired_mem, act_mem);
    append_nocopy(message);
  }
};


/** Shared memory segment does not exist.
 */
class ShmDoesNotExistException : public Exception {
 public:
  /** Constructor */
  ShmDoesNotExistException() : Exception("The given shared memory segment does not exist.") {}
};


/** The shared memory is set adress-dependend but could not be opened at the appropriate
 * address.
 */
class ShmCouldNotAttachAddrDepException : public Exception {
 public:
  /** Constructor */
  ShmCouldNotAttachAddrDepException() : Exception("Could not attach to the shared memory "
						   "segment with the appropriate address") {}
};


/** The address points out of the shared memory.
 */
class ShmAddrOutOfBoundsException : public Exception {
 public:
  /** Constructor */
  ShmAddrOutOfBoundsException() : Exception("The address you tried to transform points "
					    "out of the shared memory segment") {}
};

/** The pointer does not point inside the shared memory.
 */
class ShmPtrOutOfBoundsException : public Exception {
 public:
  /** Constructor */
  ShmPtrOutOfBoundsException() : Exception("The pointer you tried to transform does not "
					    "point inside the shared memory segment") {}
};

#endif
