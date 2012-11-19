
/***************************************************************************
 *  shm_registry.h - shared memory registry
 *
 *  Created: Sun Mar 06 12:07:50 2011
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

#ifndef __UTILS_IPC_SHM_REGISTRY_H_
#define __UTILS_IPC_SHM_REGISTRY_H_

#include <semaphore.h>
#include <list>

#define MAGIC_TOKEN_SIZE 16
#define MAXNUM_SHM_SEGMS 64
#define DEFAULT_SHM_NAME "/fawkes-shmem-registry"
#define USER_SHM_NAME "/fawkes-shmem-registry-%s"

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class SharedMemoryRegistry
{
 public:
  /** Shared memory identifier. */
  typedef struct {
    int  shmid;				/**< SysV IPC shared memory ID */
    char magic_token[MAGIC_TOKEN_SIZE];	/**< Magic token */
  } SharedMemID;

 public:
  SharedMemoryRegistry(const char *name = 0);
  ~SharedMemoryRegistry();

  std::list<SharedMemoryRegistry::SharedMemID> get_snapshot() const;

  std::list<SharedMemoryRegistry::SharedMemID>
    find_segments(const char *magic_token) const;

  void add_segment(int shmid, const char *magic_token);
  void remove_segment(int shmid);

  static void cleanup(const char *name = 0);

 private:
  /// @cond INTERNALS
  typedef struct {
    SharedMemID segments[MAXNUM_SHM_SEGMS];
  } MemInfo;
  /// @endcond

  bool  __master;
  int   __shmfd;
  char *__shm_name;

  sem_t   *__sem;
  MemInfo *__meminfo;
};


} // end namespace fawkes

#endif
