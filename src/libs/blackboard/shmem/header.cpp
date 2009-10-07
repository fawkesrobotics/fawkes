
/***************************************************************************
 *  bb_shmem_header.cpp - BlackBoard shared memory header
 *
 *  Created: Thu Oct 19 14:21:07 2006 (Anne's 25th Birthday)
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

#include <blackboard/shmem/header.h>
#include <utils/ipc/shm.h>
#include <cstddef>

namespace fawkes {

/** @class BlackBoardSharedMemoryHeader <blackboard/shmem/header.h>
 * BlackBoard Shared Memory Header.
 * This class is used identify BlackBoard shared memory headers and
 * to interact with the management data in the shared memory segment.
 * The basic options stored in the header is a version identifier
 * and pointers to the list heads of the free and allocated chunk
 * lists.
 *
 * @author Tim Niemueller
 * @see SharedMemoryHeader
 */


/** Constructor
 * @param data_size the size of the shared memory segment without the header
 *                  that should be allocated.
 * @param version The BB version to store in the shared memory segment to prevent
 *                conflicts with older software.
 */
BlackBoardSharedMemoryHeader::BlackBoardSharedMemoryHeader(size_t data_size,
							   unsigned int version)
{
  _data_size = data_size;
  _version   = version;
  data       = NULL;
}


/** Constructor.
 * @param version The BB version to store in the shared memory segment to prevent
 *                conflicts with older software.
 */
BlackBoardSharedMemoryHeader::BlackBoardSharedMemoryHeader(unsigned int version)
{
  _data_size = 0;
  _version   = version;
  data       = NULL;
}


/** Copy constructor.
 * @param h header to copy
 */
BlackBoardSharedMemoryHeader::BlackBoardSharedMemoryHeader(const BlackBoardSharedMemoryHeader *h)
{
  _data_size = h->_data_size;
  _version   = h->_version;
  data       = h->data;
}


/** Set SharedMemory instance.
 * This is needed for address conversion and must be set right after the constructor
 * call of SharedMemory!
 * @param shmem SharedMemory segment used for this header
 */
void
BlackBoardSharedMemoryHeader::set_shared_memory(SharedMemory *shmem)
{
  this->shmem = shmem;
}


/** Destructor */
BlackBoardSharedMemoryHeader::~BlackBoardSharedMemoryHeader()
{
}



SharedMemoryHeader *
BlackBoardSharedMemoryHeader::clone() const
{
  return new BlackBoardSharedMemoryHeader(this);
}


/** Check if the given shared memory segment is a Fawkes BB segment
 * @param memptr Ptr to the segment
 * @return true if the version matches, false otherwise
 */
bool
BlackBoardSharedMemoryHeader::matches(void *memptr)
{
  BlackBoardSharedMemoryHeaderData *md = (BlackBoardSharedMemoryHeaderData *)memptr;
  return (_version == md->version);
}


/** Check for equality of headers.
 * First checks if passed SharedMemoryHeader is an instance of
 * BlackBoardSharedMemoryHeader. If not returns false, otherwise it compares
 * version, data size and data pointer. If all match returns true,
 * false if any of them differs.
 * @param s shared memory header to compare to
 * @return true if the two instances identify the very same shared memory segments,
 * false otherwise
 */
bool
BlackBoardSharedMemoryHeader::operator==(const SharedMemoryHeader &s) const
{
  const BlackBoardSharedMemoryHeader *h = dynamic_cast<const BlackBoardSharedMemoryHeader *>(&s);
  if ( ! h ) {
    return false;
  } else {
    return ( (_version == h->_version) &&
	     (_data_size == h->_data_size) &&
	     (data == h->data) );
  }
}

/** Get the size of the header data.
 * @return size of the header data
 */
size_t
BlackBoardSharedMemoryHeader::size()
{
  return sizeof(BlackBoardSharedMemoryHeaderData);
}


/** Initialize shared memory segment
 * This copies basic management header data into the shared memory segment.
 * Basically sets the version and list heads to NULL
 * @param memptr pointer to the memory
 */
void
BlackBoardSharedMemoryHeader::initialize(void *memptr)
{
  data = (BlackBoardSharedMemoryHeaderData *)memptr;
  data->version         = _version;
  data->shm_addr        = memptr;
  data->free_list_head  = NULL;
  data->alloc_list_head = NULL;
}


/** Set data of this header
 * Sets the internal pointer to the shared memory header data 
 * to the data retrieved from the shared memory segment.
 * @param memptr pointer to the memory
 */
void
BlackBoardSharedMemoryHeader::set(void *memptr)
{
  data = (BlackBoardSharedMemoryHeaderData *)memptr;
}


void
BlackBoardSharedMemoryHeader::reset()
{
  data = NULL;
}


/** Data segment size.
 * @return size of the data segment without header
 */
size_t
BlackBoardSharedMemoryHeader::data_size()
{
  return _data_size;
}


/** Get the head of the free chunks list.
 * @return pointer to the free list head, local pointer, already transformed,
 * you can use this without further conversion.
 */
chunk_list_t *
BlackBoardSharedMemoryHeader::free_list_head()
{
  return (chunk_list_t *)shmem->ptr(data->free_list_head);
}


/** Get the head of the allocated chunks list.
 * @return pointer to the allocated list head, local pointer, already transformed,
 * you can use this without further conversion.
 */
chunk_list_t *
BlackBoardSharedMemoryHeader::alloc_list_head()
{
  return (chunk_list_t *)shmem->ptr(data->alloc_list_head);
}


/** Set the head of the free chunks list.
 * @param flh pointer to the new free list head, must be a pointer to the local
 * shared memory segment. Will be transformed to a shared memory address.
 */
void
BlackBoardSharedMemoryHeader::set_free_list_head(chunk_list_t *flh)
{
  data->free_list_head = (chunk_list_t *)shmem->addr(flh);
}


/** Set the head of the allocated chunks list.
 * @param alh pointer to the new allocated list head, must be a pointer to the local
 * shared memory segment. Will be transformed to a shared memory address.
 */
void
BlackBoardSharedMemoryHeader::set_alloc_list_head(chunk_list_t *alh)
{
  data->alloc_list_head = (chunk_list_t *)shmem->addr(alh);
}


/** Get BlackBoard version.
 * @return BlackBoard version
 */
unsigned int
BlackBoardSharedMemoryHeader::version() const
{
  return data->version;
}

} // end namespace fawkes
