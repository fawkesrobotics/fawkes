
/***************************************************************************
 *  shm_registry.cpp - shared memory registry
 *
 *  Created: Sun Mar 06 12:08:09 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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


#include <utils/ipc/shm_registry.h>
#include <core/exception.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cstdlib>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SharedMemoryRegistry <utils/ipc/shm_registry.h>
 * Shared memory registry.
 * This class opens a named POSIX shared memory segment, which
 * contains one instance of the MemInfo struct. It is used to detect
 * and maintain existing SysV IPC shared memory segments in a platform
 * independent way. SysV IPC shared memory segments have some advanced
 * functionality, for example reporting how many processes have
 * attached to the segment. For the registry however, we are more
 * interested in using a symbolic name which is the same for registry
 * entries. Therefore, we use this here. The struct is protected by a
 * lock implemented as a semaphore. Whenever a shared memory segment
 * is created, it is registered to the registry so others can find
 * it. On destruction, it is unregistered from the registry.
 *
 * @author Tim Niemueller
 */

/** Constructor.
 * @param master if true, the shared memory is first deleted, if it
 * existed, and then created fresh. It is resized to contain one
 * struct and initialized as an empty registry.
 * @param name name of the shared memory region. Must follow the
 * rules set by shm_open().
 */
SharedMemoryRegistry::SharedMemoryRegistry(bool master, const char *name)
{
  __master   = master;
  __shm_name = strdup(name);

  if (master) {
    // just in case...
    shm_unlink(name);
  }

  __shmfd =
    shm_open(name, O_RDWR | (master ? O_CREAT | O_EXCL : 0),
	     S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);

  if (__shmfd < 0) {
    free(__shm_name);
    throw Exception(errno, "Failed to open shared memory registry");
  }


  if (master) {
    sem_unlink(__shm_name);
    __sem = sem_open(__shm_name, O_CREAT, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, 1);
    if (__sem == SEM_FAILED) {
      close(__shmfd);
      shm_unlink(__shm_name);
      free(__shm_name);
      throw Exception(errno, "Failed to init shared memory registry semaphore");
    }
    if (ftruncate(__shmfd, sizeof(MemInfo)) != 0) {
      close(__shmfd);
      shm_unlink(__shm_name);
      sem_unlink(__shm_name);
      free(__shm_name);
      throw Exception(errno, "Failed to resize memory for shared memory registry");
    }
  } else {
    __sem = sem_open(__shm_name, 0);
    if (__sem == SEM_FAILED) {
      close(__shmfd);
      shm_unlink(__shm_name);
      free(__shm_name);
      throw Exception(errno, "Failed to init shared memory registry semaphore");
    }
  }

  __meminfo = (MemInfo *)mmap(NULL, sizeof(MemInfo), PROT_READ | PROT_WRITE,
			      MAP_SHARED, __shmfd, 0);
  if (__meminfo == MAP_FAILED) {
    close(__shmfd);
    if (master) {
      shm_unlink(__shm_name);
      sem_unlink(__shm_name);
    }
    free(__shm_name);
    throw Exception(errno, "Failed to mmap shared memory registry");
  }

  if (master) {
    memset(__meminfo, 0, sizeof(MemInfo));
  
    sem_wait(__sem);
    for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
      __meminfo->segments[i].shmid = -1;
    }
    sem_post(__sem);
  }
}


/** Destructor. */
SharedMemoryRegistry::~SharedMemoryRegistry()
{
  close(__shmfd);
  sem_close(__sem);
  if (__master) {
    shm_unlink(__shm_name);
    sem_unlink(__shm_name);
  }

  free(__shm_name);
}


/** Get a snapshot of currently registered segments.
 * @return list of all currently registered segments
 */
std::list<SharedMemoryRegistry::SharedMemID>
SharedMemoryRegistry::get_snapshot() const
{
  std::list<SharedMemID> rv;

  sem_wait(__sem);

  for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
    if (__meminfo->segments[i].shmid > 0) {
      rv.push_back(__meminfo->segments[i]);
    }
  }

  sem_post(__sem);

  return rv;
}


/** Find segments with particular magic token.
 * @param magic_token magic token to return IDs for
 * @return list of segments that currently exist with the given
 * magic token
 */
std::list<SharedMemoryRegistry::SharedMemID>
SharedMemoryRegistry::find_segments(const char *magic_token) const
{
  std::list<SharedMemID> rv;

  sem_wait(__sem);

  for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
    if ((__meminfo->segments[i].shmid > 0) &&
	(strncmp(magic_token, __meminfo->segments[i].magic_token,
		 MAGIC_TOKEN_SIZE) == 0) )
    {
      rv.push_back(__meminfo->segments[i]);
    }
  }

  sem_post(__sem);

  return rv;
}


/** Register a segment.
 * @param shmid shared memory ID of the SysV IPC segment
 * @param magic_token magic token for the new segment
 */
void
SharedMemoryRegistry::add_segment(int shmid, const char *magic_token)
{
  sem_wait(__sem);

  bool valid = false;
  for (unsigned int i = 0; !valid && i < MAXNUM_SHM_SEGMS; ++i) {
    if (__meminfo->segments[i].shmid == -1) {
      __meminfo->segments[i].shmid = shmid;
      strncpy(__meminfo->segments[i].magic_token, magic_token, MAGIC_TOKEN_SIZE);
      valid = true;
    }
  }

  sem_post(__sem);

  if (! valid) {
    throw Exception("Maximum number of shared memory segments already registered");
  }
}


/** Remove segment.
 * @param shmid shared memory ID of the segment to remove.
 */
void
SharedMemoryRegistry::remove_segment(int shmid)
{
  sem_wait(__sem);

  for (unsigned int i = 0; i < MAXNUM_SHM_SEGMS; ++i) {
    if (__meminfo->segments[i].shmid == shmid) {
      __meminfo->segments[i].shmid = -1;
    }
  }

  sem_post(__sem);
}

} // end namespace fawkes
