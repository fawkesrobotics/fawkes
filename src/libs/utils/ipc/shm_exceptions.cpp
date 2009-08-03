
/***************************************************************************
 *  shm_exceptions.cpp - exceptions thrown in shmem utils, do NOT put your own
 *                     application specific exceptions here!
 *
 *  Created: Sat Nov 11 14:15:19 2006 (on train from Hamburg to Aachen)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <utils/ipc/shm_exceptions.h>

#include <core/threading/mutex.h>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdio.h>

namespace fawkes {


/** @class ShmCouldNotAttachException shm_exceptions.h <utils/ipc/shm_exceptions.h>
 * Could not attach to shared memory segment.
 */
/** Constructor.
 * @param msg message why we could not attach
 */
ShmCouldNotAttachException::ShmCouldNotAttachException(const char *msg)
  : Exception(msg)
{
}

/** @class ShmNoHeaderException shm_exceptions.h <utils/ipc/shm_exceptions.h>
 * No shared memory header set before attach()
 */
/** Constructor. */
ShmNoHeaderException::ShmNoHeaderException()
  : Exception("No SharedMemoryHeader, cannot attach")
{
}


/** @class ShmInconsistentSegmentSizeException shm_exceptions.h <utils/ipc/shm_exceptions.h>
 * Memory size does not match
 */
/** Constructor
 * @param desired_mem The exepcted memory size
 * @param act_mem The actual memory size
 */
ShmInconsistentSegmentSizeException::ShmInconsistentSegmentSizeException(unsigned int desired_mem,
									 unsigned int act_mem)
  : Exception("Inconsistent shared mem segment found in memory "
	      "(memory size does not match, desired: %u, actual: %u)",
	      desired_mem, act_mem)
{
}


/** @class ShmDoesNotExistException shm_exceptions.h <utils/ipc/shm_exceptions.h>
 * Shared memory segment does not exist.
 */
/** Constructor */
ShmDoesNotExistException::ShmDoesNotExistException()
  : Exception("The given shared memory segment does not exist.")
{
}


/** @class ShmCouldNotAttachAddrDepException shm_exceptions.h <utils/ipc/shm_exceptions.h>
 * The shared memory is set adress-dependend but could not be opened at the appropriate
 * address.
 */
/** Constructor. */
ShmCouldNotAttachAddrDepException::ShmCouldNotAttachAddrDepException()
  : Exception("Could not attach to the shared memory "
	      "segment with the appropriate address")
{
}


/** @class ShmAddrOutOfBoundsException shm_exceptions.h <utils/ipc/shm_exceptions.h>
 * The address points out of the shared memory.
 */
/** Constructor. */
ShmAddrOutOfBoundsException::ShmAddrOutOfBoundsException()
  : Exception("The address you tried to transform points "
	      "out of the shared memory segment")
{
}


/** @class ShmPtrOutOfBoundsException shm_exceptions.h <utils/ipc/shm_exceptions.h>
 * The pointer does not point inside the shared memory.
 */
/** Constructor. */
ShmPtrOutOfBoundsException::ShmPtrOutOfBoundsException()
  : Exception("The pointer you tried to transform does not "
	      "point inside the shared memory segment")
{
}

} // end namespace fawkes
