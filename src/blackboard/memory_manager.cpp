
/***************************************************************************
 *  memory_manager.cpp - BlackBoard memory manager
 *
 *  Generated: Sat Sep 23 16:03:40 2006 (INSITE 2006, Joburg, South Africa)
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

#include <blackboard/memory_manager.h>
#include <blackboard/exceptions.h>

#include <core/exception.h>
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>

#include <utils/ipc/shm.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <string>

/** If a free chunk is allocated it may be split up into an allocated
 * and a new free chunk. This value determines when this is done. If
 * there are at least this many data bytes (without the list header)
 * then the chunk is split, otherwise it is fully allocated with
 * overhanging bytes.
 */
#define BBMM_MIN_FREE_CHUNK_SIZE sizeof(chunk_list_t)

/** BlackBoard Shared Memory Header
 * This class is used identify BlackBoard shared memory headers and
 * to interact with the management data in the shared memory segment.
 * The basic options stored in the header is a version identifier
 * and pointers to the list heads of the free and allocated chunk
 * lists.
 *
 * @author Tim Niemueller
 * @see SharedMemoryHeader
 */
class BlackBoardSharedMemoryHeader : public SharedMemoryHeader
{
 private:
  /** This struct determines the header in the shared memory segment
   */
  typedef struct {
    unsigned int  version;		/**< version of the BB */
    chunk_list_t *free_list_head;	/**< pointer to the free chunks list head */
    chunk_list_t *alloc_list_head;	/**< pointer to the allocated chunks list head */
  } BlackBoardSharedMemoryHeaderData;

 public:
  /** Constructor
   * @param data_size the size of the shared memory segment without the header
   *                  that should be allocated.
   * @param version The BB version to store in the shared memory segment to prevent
   *                conflicts with older software.
   */
  BlackBoardSharedMemoryHeader(unsigned int data_size, unsigned int version)
  {
    this->data_size      = data_size;
    this->version        = version;
    data = NULL;
  }

  /** Destructor */
  virtual ~BlackBoardSharedMemoryHeader()
  {
  }

  /** Check if the given shared memory segment is a Fawkes BB segment
   * @param memptr Ptr to the segment
   * @return true if the version matches, false otherwise
   */
  virtual bool         matches(void *memptr)
  {
    BlackBoardSharedMemoryHeaderData *md = (BlackBoardSharedMemoryHeaderData *)memptr;
    return (version == md->version);
  }

  /** Get the size of the header data.
   * @return size of the header data
   */
  virtual unsigned int size()
  {
    return sizeof(BlackBoardSharedMemoryHeaderData);
  }

  /** Initialize shared memory segment
   * This copies basic management header data into the shared memory segment.
   * Basically sets the version and list heads to NULL
   * @param memptr pointer to the memory
   */
  virtual void         initialize(void *memptr)
  {
    data = (BlackBoardSharedMemoryHeaderData *)memptr;
    data->version = version;
    data->free_list_head = NULL;
    data->alloc_list_head = NULL;
  }

  /** Set data of this header
   * Sets the internal pointer to the shared memory header data 
   * to the data retrieved from the shared memory segment.
   * @param memptr pointer to the memory
   */
  virtual void         set(void *memptr)
  {
    data = (BlackBoardSharedMemoryHeaderData *)memptr;
  }

  /** Data segment size.
   * @return size of the data segment without header
   */
  virtual unsigned int dataSize()
  {
    return data_size;
  }

  /** Get the head of the free chunks list.
   * @return pointer to the free list head
   */
  chunk_list_t * getFreeListHead()
  {
    return data->free_list_head;
  }

  /** Get the head of the allocated chunks list.
   * @return pointer to the allocated list head
   */
  chunk_list_t * getAllocListHead()
  {
    return data->alloc_list_head;
  }

  /** Set the head of the free chunks list.
   * @param flh pointer to the new free list head
   */
  void setFreeListHead(chunk_list_t *flh)
  {
    data->free_list_head = flh;
  }

  /** Set the head of the allocated chunks list.
   * @param alh pointer to the new allocated list head
   */
  void setAllocListHead(chunk_list_t *alh)
  {
    data->alloc_list_head = alh;
  }

 private:
  unsigned int data_size;
  unsigned int version;
  BlackBoardSharedMemoryHeaderData *data;
};


/** @class BlackBoardMemoryManager blackboard/memory_manager.h
 * BlackBoard memory manager
 * This class is used by the BlackBoard to manage the memory in the shared memory
 * segment. A simple strategy is used for memory management as the expected use case
 * is rather simple as well.
 *
 * The memory is allocated as one big chunk of contiguous memory. Inside this
 * chunk the memory manager handles the smaller chunks that are allocated in this
 * region. The chunk is allocated as shared memory segment to allow for multi-process
 * usage of the memory.
 *
 * The memory is organized in two separate lists. The one is the free chunks list
 * and the other one the allocated chunks list. After startup the allocated
 * chunks list is empty while the free chunks list contains one and only one
 * big chunk of free memory that contains the whole data segment.
 *
 * When memory is allocated the smallest chunk that is big enough for the requested
 * chunk is used. It is then removed from the free list. If the chunk is big enough
 * to hold another chunk of memory (the remaining size can accomodate the header
 * and at least as many bytes as the header is in size) the chunk is split into an
 * exactly fitting allocated chunk and a remaining free chunk. The chunks are then
 * added to the appropriate lists. If there is more memory then requested but
 * not enough memory to make it a new free chunk the allocated chunk is enlarged
 * to fill the whole chunk. The additional bytes are recorded as overhanging bytes.
 *
 * When memory is freed the chunk is removed from the allocated chunks list and
 * added to the free chunks list. Then the list is cleaned up and adjacent regions
 * of free memory are merged to one. Afterwards the free chunks list will contain
 * non-ajdacent free memory regions of maximum size between allocated chunks.
 *
 * The memory manager is thread-safe as all appropriate operations are protected
 * by a mutex.
 *
 * The memory manager has also been prepared for multi-process usage of the
 * shared memory region.   but up to now only one process may use the shared
 * memory segment.
 *
 * @todo implement multi-process feature
 *
 * @author Tim Niemueller
 * @see SharedMemory
 * @see Mutex
 */


/** Constructor
 * @param memsize the size of the shared memory segment (data without header)
 * that is being managed.
 * @param version version of the BlackBoard
 * @param master master mode, this memory manager has to be owner of shared memory segment
 * @param shmem_token shared memory token, passed to SharedMemory
 * @exception BBMemMgrNotMasterException A matching shared memory segment
 * has already been created.
 * @see SharedMemory::SharedMemory()
 */
BlackBoardMemoryManager::BlackBoardMemoryManager(unsigned int memsize,
						 unsigned int version,
						 bool master,
						 const char *shmem_token)
{

  this->memsize = memsize;

  // open shared memory segment, if it exists try to aquire exclusive
  // semaphore, if that fails, throw an exception
  
  shmem_header = new BlackBoardSharedMemoryHeader(memsize, version);
  shmem = new SharedMemory(shmem_token, shmem_header,
			   /* read only   */ false,
			   /* create      */ true,
			   /* dest on del */ true);

  if ( master && ! shmem->isCreator() ) {
    // this might mean trouble, we are throw general exception for now,
    // needs more useful handling later
    throw BBMemMgrNotMasterException("Not owner of shared memory segment");
  }

  // protect memory, needed for list operations in memory, otherwise
  // we will have havoc and insanity
  shmem->addSemaphore();

  chunk_list_t *f = (chunk_list_t *)shmem->getMemPtr();
  f->ptr  = (char *)f + sizeof(chunk_list_t);
  f->size = memsize - sizeof(chunk_list_t);
  f->overhang = 0;
  f->next = NULL;
  shmem_header->setFreeListHead(f);
  shmem_header->setAllocListHead(NULL);

  mutex = new Mutex();
}


/** Destructor */
BlackBoardMemoryManager::~BlackBoardMemoryManager()
{
  // close shared memory segment, kill semaphore
  delete shmem;
  delete shmem_header;
  delete mutex;
}


/** Allocate memory.
 * This will allocate memory in the shared memory segment. The strategy is described
 * in the class description.
 * @exception OutOfMemoryException thrown if not enough free memory is available to
 *                                 accommodate a chunk of the desired size
 * @param num_bytes number of bytes to allocate
 * @return pointer to the memory chunk
 */
void *
BlackBoardMemoryManager::alloc(unsigned int num_bytes)
{
  mutex->lock();
  shmem->lock();
  // search for smallest chunk just big enough for desired size
  chunk_list_t *l = shmem_header->getFreeListHead();

  // Note: free chunks list sorted ascending by ptr
  chunk_list_t *f = NULL;
  while ( l ) {
    if ( (l->size >= num_bytes) && // chunk is big enough
	 ( (f == NULL) || (l->size < f->size) ) ) { // no chunk found or current chunk smaller
      f = l;
    }
    l = l->next;
  }

  if ( f == NULL ) {
    // Doh, did not find chunk
    shmem->unlock();
    mutex->unlock();
    throw OutOfMemoryException("BlackBoard ran out of memory");
  }

  // remove chunk from free_list
  shmem_header->setFreeListHead( list_remove(shmem_header->getFreeListHead(), f) );

  /*
  // only chunk's semaphore
  if ( ptr_sems.find( f->ptr ) != ptr_sems.end() ) {
    SemaphoreSet *s = ptr_sems[f->ptr];
    delete s;
    ptr_sems.erase( f->ptr );
    f->semset_key = 0;
  }
  */

  // our old free list chunk is now our new alloc list chunk
  // check if there is free space beyond the requested size that makes it worth
  // entering it into the free list
  if ( f->size >= (num_bytes + BBMM_MIN_FREE_CHUNK_SIZE + sizeof(chunk_list_t)) ) {
    // we will have a new free chunk afterwards
    chunk_list_t *nfc = (chunk_list_t *)((char *)f + sizeof(chunk_list_t) + num_bytes);
    nfc->ptr = (char *)nfc + sizeof(chunk_list_t);
    nfc->size = f->size - num_bytes - sizeof(chunk_list_t);
    nfc->overhang = 0;
    
    shmem_header->setFreeListHead( list_add(shmem_header->getFreeListHead(), nfc) );

    f->size = num_bytes;
  } else {
    // chunk is too small for another free chunk, now we have allocated but unusued
    // space, this is ok but not desireable
    // this is only informational!
    f->overhang = f->size - num_bytes;
  }

  // alloc new chunk
  shmem_header->setAllocListHead( list_add(shmem_header->getAllocListHead(), f) );

  shmem->unlock();
  mutex->unlock();
  return f->ptr;
}


/** Free a memory chunk.
 * Frees a previously allocated chunk. Not that you have to give the exact pointer
 * that was returned by alloc(). You may not give a pointer inside a memory chunk or
 * even worse outside of it! See the class description for a brief description of
 * the strategy used.
 * @param chunk_ptr pointer to the chunk of memory
 * @exception BlackBoardMemMgrInvalidPointerException a pointer that has not been
 * previously returned by alloc() has been given and could not be found in the
 * allocated chunks list.
 */
void
BlackBoardMemoryManager::free(void *chunk_ptr)
{
  mutex->lock();
  shmem->lock();

  // find chunk in alloc_chunks
  chunk_list_t *ac = list_find_ptr(shmem_header->getAllocListHead(), chunk_ptr);
  if ( ac == NULL ) {
    throw BlackBoardMemMgrInvalidPointerException();
  }

  // remove from alloc_chunks
  shmem_header->setAllocListHead( list_remove(shmem_header->getAllocListHead(), ac) );

  // reclaim as free memory
  ac->overhang = 0;
  shmem_header->setFreeListHead( list_add(shmem_header->getFreeListHead(), ac) );

  // merge adjacent regions
  cleanup_free_chunks();

  shmem->unlock();
  mutex->unlock();
}


/** Check memory consistency.
 * This method checks the consistency of the memory segment. It controls whether
 * all the memory is covered by the free and allocated chunks lists and if there is
 * no unmanaged memory between chunks.
 * @exception BBInconsistentMemoryException thrown if the memory segment has been
 * corrupted. Contains descriptive message.
 */
void
BlackBoardMemoryManager::check()
{
  chunk_list_t *f = shmem_header->getFreeListHead();
  chunk_list_t *a = shmem_header->getAllocListHead();
  chunk_list_t *t = NULL;

  unsigned int mem = 0;

  // we crawl through the memory and analyse if the chunks are continuous,
  // assumption: chunk list sorted ascending by ptr
  while ( f || a ) {
    if ( f == NULL ) {
      mem += a->size + sizeof(chunk_list_t);
      if ( a->next ) {
	// check if a is continuous
	void *next = (char *)a->ptr + a->size + sizeof(chunk_list_t);
	if ( next != a->next->ptr ) {
	  throw BBInconsistentMemoryException("non-contiguos allocated memory");	
	}
      }
      a = a->next;
    } else if ( a == NULL ) {
      mem += f->size + sizeof(chunk_list_t);
      if ( f->next ) {
	// check if f is continuous
	void *next = (char *)f->ptr + f->size + sizeof(chunk_list_t);
	if ( next != f->next->ptr ) {
	  throw BBInconsistentMemoryException("non-contiguos allocated memory");	
	}
      }
      f = f->next;
    } else if ( f->ptr == a->ptr ) {
      throw BBInconsistentMemoryException("ptr cannot be free and allocated at the same time");
    } else if ( f->ptr < a->ptr ) {
      mem += f->size + sizeof(chunk_list_t);
      void *next = (char *)f->ptr + f->size;
      t = f->next;
      if ( (next != t) && (next != a) ) {
	throw BBInconsistentMemoryException("there are unallocated bytes between chunks (f)");
      }
      f = f->next;
    } else {
      mem += a->size + sizeof(chunk_list_t);
      void *next = (char *)a->ptr + a->size;
      t = a->next;
      if ( (next != t) && (next != f) ) {
	throw BBInconsistentMemoryException("there are unallocated bytes between chunks (a)");
      }
      a = a->next;
    }
  }

  if ( mem != memsize ) {
    throw BBInconsistentMemoryException("unmanaged memory found, managed memory size != total memory size");
  }
}


/** Print out info about free chunks.
 * Prints out a formatted list of free chunks.
 */
void
BlackBoardMemoryManager::printFreeChunksInfo()
{
  list_print_info( shmem_header->getFreeListHead() );
}


/** Print out info about allocated chunks.
 * Prints out a formatted list of allocated chunks.
 */
void
BlackBoardMemoryManager::printAllocatedChunksInfo()
{
  list_print_info( shmem_header->getAllocListHead() );
}


/** Prints out performance info.
 * This will print out information about the number of free and allocated chunks,
 * the maximum free and allocated chunk size and the number of overhanging bytes
 * (see class description about overhanging bytes).
 */
void
BlackBoardMemoryManager::printPerformanceInfo()
{
  printf("free chunks: %6u, alloc chunks: %6u, max free: %10u, max alloc: %10u, overhang: %10u\n",
	 list_length( shmem_header->getFreeListHead()),
	 list_length( shmem_header->getAllocListHead()),
	 getMaxFreeSize(), getMaxAllocatedSize(), getOverhangSize());
}


/** Get maximum allocatable memory size.
 * This method gives information about the maximum free chunk size and thus
 * the maximum of memory that can be allocated in one chunk.
 * @return maximum free chunk size
 */
unsigned int
BlackBoardMemoryManager::getMaxFreeSize()
{
  chunk_list_t *m = list_get_biggest( shmem_header->getFreeListHead() );
  if ( m == NULL ) {
    return 0;
  } else {
    return m->size;
  }
}


/** Get maximum alloced memory size.
 * This method gives information about the maximum allocated chunk size and thus
 * the maximum of memory that has been be allocated in one chunk.
 * @return maximum allocated chunk size
 */
unsigned int
BlackBoardMemoryManager::getMaxAllocatedSize()
{
  chunk_list_t *m = list_get_biggest( shmem_header->getAllocListHead() );
  if ( m == NULL ) {
    return 0;
  } else {
    return m->size;
  }
}


/** Get number of overhanging bytes.
 * The number of overhanging bytes. See class description for more info about
 * overhanging bytes.
 * @return number of overhanging bytes
 */
unsigned int
BlackBoardMemoryManager::getOverhangSize()
{
  unsigned int overhang = 0;
  chunk_list_t *a = shmem_header->getAllocListHead();
  while ( a ) {
    overhang += a->overhang;
    a = a->next;
  }
  return overhang;
}


/** Cleanup and merge free chunks.
 * This will merge adjacent free chunks into one big chunk. After this method ran it
 * is guaranteed that the maximum available memory resides in one chunk.
 */
void
BlackBoardMemoryManager::cleanup_free_chunks()
{
  bool modified = true;
  chunk_list_t *l;

  while (modified) {
    modified = false;
    l = shmem_header->getFreeListHead();
    while ( l && l->next) {
      if ( ((char *)l->ptr + l->size + sizeof(chunk_list_t)) == l->next->ptr ) {
	// re-unite
	l->size += l->next->size + sizeof(chunk_list_t);
	l->next = l->next->next;
	modified = true;
      }
      l = l->next;
    }
  }
}


/** Remove an element from a list.
 * @param list list to remove the element from
 * @param rmel element to remove
 * @return the head of the new resulting list
 * @exception NullPointerException thrown if list or rmel equals NULL
 */
chunk_list_t *
BlackBoardMemoryManager::list_remove(chunk_list_t *list, chunk_list_t *rmel)
{
  if ( list == NULL )
    throw NullPointerException("BlackBoardMemoryManager::list_remove: list == NULL");
  if ( rmel == NULL )
    throw NullPointerException("BlackBoardMemoryManager::list_remove: rmel == NULL");


  chunk_list_t *new_head = list;
  chunk_list_t *l = list;
  chunk_list_t *p = NULL;

  while ( l ) {
    if ( l == rmel ) {
      // found element, now remove
      if ( p ) {
	// we have a predecessor
	p->next = l->next;
      } else {
	// new head
	new_head = l->next;
      }
      break;
    }
    p = l;
    l = l->next;
  }

  return new_head;
}


/** Add an element to a list.
 * @param list list to add the element to
 * @param rmel element to add
 * @return the head of the new resulting list
 * @exception NullPointerException thrown if addel equals NULL
 */
chunk_list_t *
BlackBoardMemoryManager::list_add(chunk_list_t *list, chunk_list_t *addel)
{
  if ( addel == NULL )
    throw NullPointerException("BlackBoardMemoryManager::list_add: addel == NULL");

  chunk_list_t *new_head = list;
  chunk_list_t *l = list;
  chunk_list_t *p = NULL;

  while ( l ) {
    if ( addel->ptr < l->ptr ) {
      // add it here
      addel->next = l;
      if ( p != NULL ) {
	// predecessor needs new successor
	// before: p->next == l
	p->next = addel;
      } else {
	new_head = addel;
      }
      // used as condition below
      l = addel;
      break;
    } else {
      p = l;
      l = l->next;
    }
  }

  // if l is not addel it has not yet been added
  if ( l != addel ) {
    // p is last element of list and != NULL
    addel->next = NULL;
    if ( p ) {
      p->next     = addel;
    } else {
      new_head = addel;
    }
  }

  return new_head;
}


/** Find a chunk by ptr.
 * @param list list to search
 * @param ptr Pointer to search for
 * @return the chunk that points to ptr or NULL if not found
 */
chunk_list_t *
BlackBoardMemoryManager::list_find_ptr(chunk_list_t *list, void *ptr)
{
  chunk_list_t *l = list;
  while ( l ) {
    if ( l->ptr == ptr ) {
      // found it
      return l;
    } else {
      l = l->next;
    }
  }
  return l;
}


/** Print info about chunks in list.
 * Will print information about chunks in list to stdout. Will give pointer as hexadezimal
 * number, size and overhanging bytes of chunk
 * @param list list with chunks to print
 */
void
BlackBoardMemoryManager::list_print_info(chunk_list_t *list)
{
  chunk_list_t *l = list;
  unsigned int i = 0;

  while ( l ) {
    printf("Chunk %3u:  0x%x   size=%10u bytes   overhang=%10u bytes\n",
	   ++i, (unsigned int)l->ptr, l->size, l->overhang);
    l = l->next;
  }

}


/** Get length of list.
 * @param list list to count
 * @return length of list
 */
unsigned int
BlackBoardMemoryManager::list_length(chunk_list_t *list)
{
  unsigned int l = 0;
  while ( list ) {
    ++l;
    list = list->next;
  }
  return l;
}


/** Get biggest chunk from list.
 * @param list list to search
 * @return biggest chunk in list
 */
chunk_list_t *
BlackBoardMemoryManager::list_get_biggest(chunk_list_t *list)
{
  chunk_list_t *b = list;
  while ( list ) {
    if ( list->size > b->size ) {
      b = list;
    }
    list = list->next;
  }

  return b;
}
