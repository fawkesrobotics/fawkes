
/***************************************************************************
 *  bb_shmem_header.cpp - BlackBoard shared memory header
 *
 *  Created: Thu Oct 19 14:21:07 2006 (Anne's 25th Birthday)
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

#include <blackboard/shmem_header.h>
#include <utils/ipc/shm.h>
#include <cstddef>


/** @class BlackBoardSharedMemoryHeader blackboard/shmem_header.h
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
BlackBoardSharedMemoryHeader::BlackBoardSharedMemoryHeader(unsigned int data_size,
							   unsigned int version)
{
  this->data_size      = data_size;
  this->version        = version;
  data = NULL;
}


/** Set SharedMemory instance.
 * This is needed for address conversion and must be set right after the constructor
 * call of SharedMemory!
 * @param shmem SharedMemory segment used for this header
 */
void
BlackBoardSharedMemoryHeader::setSharedMemory(SharedMemory *shmem)
{
  this->shmem = shmem;
}


/** Destructor */
BlackBoardSharedMemoryHeader::~BlackBoardSharedMemoryHeader()
{
}

/** Check if the given shared memory segment is a Fawkes BB segment
 * @param memptr Ptr to the segment
 * @return true if the version matches, false otherwise
 */
bool
BlackBoardSharedMemoryHeader::matches(void *memptr)
{
  BlackBoardSharedMemoryHeaderData *md = (BlackBoardSharedMemoryHeaderData *)memptr;
  return (version == md->version);
}

/** Get the size of the header data.
 * @return size of the header data
 */
unsigned int
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
  data->version         = version;
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

/** Data segment size.
 * @return size of the data segment without header
 */
unsigned int
BlackBoardSharedMemoryHeader::dataSize()
{
  return data_size;
}

/** Get the head of the free chunks list.
 * @return pointer to the free list head, local pointer, already transformed,
 * you can use this without further conversion.
 */
chunk_list_t *
BlackBoardSharedMemoryHeader::getFreeListHead()
{
  return (chunk_list_t *)shmem->ptr(data->free_list_head);
}

/** Get the head of the allocated chunks list.
 * @return pointer to the allocated list head, local pointer, already transformed,
 * you can use this without further conversion.
 */
chunk_list_t *
BlackBoardSharedMemoryHeader::getAllocListHead()
{
  return (chunk_list_t *)shmem->ptr(data->alloc_list_head);
}

/** Set the head of the free chunks list.
 * @param flh pointer to the new free list head, must be a pointer to the local
 * shared memory segment. Will be transformed to a shared memory address.
 */
void
BlackBoardSharedMemoryHeader::setFreeListHead(chunk_list_t *flh)
{
  data->free_list_head = (chunk_list_t *)shmem->addr(flh);
}

/** Set the head of the allocated chunks list.
 * @param alh pointer to the new allocated list head, must be a pointer to the local
 * shared memory segment. Will be transformed to a shared memory address.
 */
void
BlackBoardSharedMemoryHeader::setAllocListHead(chunk_list_t *alh)
{
  data->alloc_list_head = (chunk_list_t *)shmem->addr(alh);
}
