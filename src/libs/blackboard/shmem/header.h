
/***************************************************************************
 *  bb_shmem_header.h - BlackBoard shared memory header
 *
 *  Created: Thu Oct 19 14:19:06 2006 (Anne's 25th Birthday)
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

#ifndef __BLACKBOARD_SHMEM_HEADER_H_
#define __BLACKBOARD_SHMEM_HEADER_H_

#include <utils/ipc/shm.h>
#include <blackboard/internal/memory_manager.h>

namespace fawkes {

class SharedMemory;

class BlackBoardSharedMemoryHeader : public SharedMemoryHeader
{
 private:
  /** This struct determines the header in the shared memory segment
   */
  typedef struct {
    unsigned int  version;		/**< version of the BB */
    void         *shm_addr;             /**< base addr of shared memory */
    chunk_list_t *free_list_head;	/**< offset of the free chunks list head */
    chunk_list_t *alloc_list_head;	/**< offset of the allocated chunks list head */
  } BlackBoardSharedMemoryHeaderData;

 public:
  BlackBoardSharedMemoryHeader(unsigned int version);
  BlackBoardSharedMemoryHeader(size_t data_size, unsigned int version);
  BlackBoardSharedMemoryHeader(const BlackBoardSharedMemoryHeader *h);
  virtual ~BlackBoardSharedMemoryHeader();
  void           set_shared_memory(SharedMemory *shmem);
  virtual bool   matches(void *memptr);
  virtual size_t size();
  virtual void   initialize(void *memptr);
  virtual void   set(void *memptr);
  virtual void   reset();
  virtual size_t data_size();
  virtual SharedMemoryHeader * clone() const;
  virtual bool   operator==(const fawkes::SharedMemoryHeader &s) const;
  chunk_list_t * free_list_head();
  chunk_list_t * alloc_list_head();
  void set_free_list_head(chunk_list_t *flh);
  void set_alloc_list_head(chunk_list_t *alh);

  unsigned int version() const;

 private:
  size_t       _data_size;
  unsigned int _version;
  BlackBoardSharedMemoryHeaderData *data;
  SharedMemory                     *shmem;
};

} // end namespace fawkes

#endif
