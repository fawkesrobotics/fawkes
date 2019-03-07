
/***************************************************************************
 *  memory_manager.cpp - BlackBoard memory manager
 *
 *  Created: Sat Sep 23 16:03:40 2006 (INSITE 2006, Joburg, South Africa)
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/exceptions.h>
#include <blackboard/internal/memory_manager.h>
#include <blackboard/shmem/header.h>
#include <core/exception.h>
#include <core/exceptions/software.h>
#include <core/exceptions/system.h>
#include <core/threading/mutex.h>
#include <sys/mman.h>
#include <utils/ipc/shm.h>
#include <utils/ipc/shm_exceptions.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

/** If a free chunk is allocated it may be split up into an allocated
 * and a new free chunk. This value determines when this is done. If
 * there are at least this many data bytes (without the list header)
 * then the chunk is split, otherwise it is fully allocated with
 * overhanging bytes.
 */
#define BBMM_MIN_FREE_CHUNK_SIZE sizeof(chunk_list_t)

// shortcuts
#define chunk_ptr(a) (shmem_ ? (chunk_list_t *)shmem_->ptr(a) : a)
#define chunk_addr(a) (shmem_ ? (chunk_list_t *)shmem_->addr(a) : a)

namespace fawkes {

/** @class BlackBoardMemoryManager <blackboard/internal/memory_manager.h>
 * BlackBoard memory manager.
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

/** Heap Memory Constructor.
 * Constructs a memory segment on the heap.
 * @param memsize memory size
 */
BlackBoardMemoryManager::BlackBoardMemoryManager(size_t memsize)
{
	shmem_        = NULL;
	shmem_header_ = NULL;
	memsize_      = memsize;
	memory_       = malloc(memsize);
	mutex_        = new Mutex();
	master_       = true;

	// Lock memory to RAM to avoid swapping
	mlock(memory_, memsize_);

	chunk_list_t *f = (chunk_list_t *)memory_;
	f->ptr          = (char *)f + sizeof(chunk_list_t);
	f->size         = memsize_ - sizeof(chunk_list_t);
	f->overhang     = 0;
	f->next         = NULL;

	free_list_head_  = f;
	alloc_list_head_ = NULL;
}

/** Shared Memory Constructor
 * @param memsize the size of the shared memory segment (data without header)
 * that is being managed.
 * @param version version of the BlackBoard
 * @param master master mode, this memory manager has to be owner of shared memory segment
 * @param shmem_token shared memory token, passed to SharedMemory
 * @exception BBMemMgrNotMasterException A matching shared memory segment
 * has already been created.
 * @see SharedMemory::SharedMemory()
 */
BlackBoardMemoryManager::BlackBoardMemoryManager(size_t       memsize,
                                                 unsigned int version,
                                                 bool         master,
                                                 const char * shmem_token)
{
	memory_  = NULL;
	memsize_ = memsize;
	master_  = master;

	// open shared memory segment, if it exists try to aquire exclusive
	// semaphore, if that fails, throw an exception

	shmem_header_ = new BlackBoardSharedMemoryHeader(memsize_, version);
	try {
		shmem_ = new SharedMemory(shmem_token,
		                          shmem_header_,
		                          /* read only   */ false,
		                          /* create      */ master_,
		                          /* dest on del */ master_);
		shmem_header_->set_shared_memory(shmem_);
	} catch (ShmCouldNotAttachException &e) {
		delete shmem_header_;
		throw BBMemMgrCannotOpenException();
	}

	if (!shmem_->is_valid()) {
		shmem_->set_destroy_on_delete(false);
		delete shmem_;
		delete shmem_header_;
		throw BBMemMgrCannotOpenException();
	}

	if (master && !shmem_->is_creator()) {
		// this might mean trouble, we throw an exception if we are not master but
		// this was requested
		shmem_->set_destroy_on_delete(false);
		delete shmem_;
		delete shmem_header_;
		throw BBNotMasterException("Not owner of shared memory segment");
	}

	// printf("Shared memory base pointer: 0x%x\n", (size_t)shmem->getMemPtr());

	if (master) {
		// protect memory, needed for list operations in memory, otherwise
		// we will have havoc and insanity
		shmem_->add_semaphore();

		// This should not be swapped. Will only worked with greatly extended
		// ressource limit for this process!
		shmem_->set_swapable(false);

		chunk_list_t *f = (chunk_list_t *)shmem_->memptr();
		f->ptr          = shmem_->addr((char *)f + sizeof(chunk_list_t));
		f->size         = memsize_ - sizeof(chunk_list_t);
		f->overhang     = 0;
		f->next         = NULL;

		shmem_header_->set_free_list_head(f);
		shmem_header_->set_alloc_list_head(NULL);
	}

	mutex_ = new Mutex();
}

/** Destructor */
BlackBoardMemoryManager::~BlackBoardMemoryManager()
{
	// close shared memory segment, kill semaphore
	delete shmem_;
	delete shmem_header_;
	if (memory_) {
		::free(memory_);
	}
	delete mutex_;
}

/** Allocate memory.
 * This will allocate memory in the shared memory segment. The strategy is described
 * in the class description. Note: this method does NOT lock the shared memory
 * system. Chaos and havoc will come down upon you if you do not ensure locking!
 * @exception OutOfMemoryException thrown if not enough free memory is available to
 *                                 accommodate a chunk of the desired size
 * @param num_bytes number of bytes to allocate
 * @return pointer to the memory chunk
 */
void *
BlackBoardMemoryManager::alloc_nolock(unsigned int num_bytes)
{
	// search for smallest chunk just big enough for desired size
	chunk_list_t *l = shmem_ ? shmem_header_->free_list_head() : free_list_head_;

	// Note: free chunks list sorted ascending by ptr
	chunk_list_t *f = NULL;
	while (l) {
		if ((l->size >= num_bytes) &&               // chunk is big enough
		    ((f == NULL) || (l->size < f->size))) { // no chunk found or current chunk smaller
			f = l;
		}
		l = chunk_ptr(l->next);
	}

	if (f == NULL) {
		// Doh, did not find chunk
		throw OutOfMemoryException("BlackBoard ran out of memory");
	}

	// remove chunk from free_list
	if (shmem_) {
		shmem_header_->set_free_list_head(list_remove(shmem_header_->free_list_head(), f));
	} else {
		free_list_head_ = list_remove(free_list_head_, f);
	}

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
	if (f->size >= (num_bytes + BBMM_MIN_FREE_CHUNK_SIZE + sizeof(chunk_list_t))) {
		// we will have a new free chunk afterwards
		chunk_list_t *nfc = (chunk_list_t *)((char *)f + sizeof(chunk_list_t) + num_bytes);
		nfc->ptr          = shmem_ ? shmem_->addr((char *)nfc + sizeof(chunk_list_t))
		                  : (char *)nfc + sizeof(chunk_list_t);
		nfc->size     = f->size - num_bytes - sizeof(chunk_list_t);
		nfc->overhang = 0;

		if (shmem_) {
			shmem_header_->set_free_list_head(list_add(shmem_header_->free_list_head(), nfc));
		} else {
			free_list_head_ = list_add(free_list_head_, nfc);
		}

		f->size = num_bytes;
	} else {
		// chunk is too small for another free chunk, now we have allocated but unusued
		// space, this is ok but not desireable
		// this is only informational!
		f->overhang = f->size - num_bytes;
	}

	// alloc new chunk
	if (shmem_) {
		shmem_header_->set_alloc_list_head(list_add(shmem_header_->alloc_list_head(), f));
		return shmem_->ptr(f->ptr);
	} else {
		alloc_list_head_ = list_add(alloc_list_head_, f);
		return f->ptr;
	}
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
	void *ptr;
	mutex_->lock();
	if (shmem_)
		shmem_->lock_for_write();
	try {
		ptr = alloc_nolock(num_bytes);
	} catch (Exception &e) {
		if (shmem_)
			shmem_->unlock();
		mutex_->unlock();
		throw;
	}
	if (shmem_)
		shmem_->unlock();
	mutex_->unlock();
	return ptr;
}

/** Free a memory chunk.
 * Frees a previously allocated chunk. Not that you have to give the exact pointer
 * that was returned by alloc(). You may not give a pointer inside a memory chunk or
 * even worse outside of it! See the class description for a brief description of
 * the strategy used.
 * @param ptr pointer to the chunk of memory
 * @exception BlackBoardMemMgrInvalidPointerException a pointer that has not been
 * previously returned by alloc() has been given and could not be found in the
 * allocated chunks list.
 */
void
BlackBoardMemoryManager::free(void *ptr)
{
	mutex_->lock();
	if (shmem_) {
		shmem_->lock_for_write();

		// find chunk in alloc_chunks
		chunk_list_t *ac = list_find_ptr(shmem_header_->alloc_list_head(), chunk_addr(ptr));
		if (ac == NULL) {
			throw BlackBoardMemMgrInvalidPointerException();
		}

		// remove from alloc_chunks
		shmem_header_->set_alloc_list_head(list_remove(shmem_header_->alloc_list_head(), ac));

		// reclaim as free memory
		ac->overhang = 0;
		shmem_header_->set_free_list_head(list_add(shmem_header_->free_list_head(), ac));

		// merge adjacent regions
		cleanup_free_chunks();

		shmem_->unlock();
	} else {
		// find chunk in alloc_chunks
		chunk_list_t *ac = list_find_ptr(alloc_list_head_, ptr);
		if (ac == NULL) {
			throw BlackBoardMemMgrInvalidPointerException();
		}

		// remove from alloc_chunks
		alloc_list_head_ = list_remove(alloc_list_head_, ac);

		// reclaim as free memory
		ac->overhang    = 0;
		free_list_head_ = list_add(free_list_head_, ac);

		// merge adjacent regions
		cleanup_free_chunks();
	}

	mutex_->unlock();
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
	chunk_list_t *f = shmem_ ? shmem_header_->free_list_head() : free_list_head_;
	chunk_list_t *a = shmem_ ? shmem_header_->alloc_list_head() : alloc_list_head_;
	chunk_list_t *t = NULL;

	unsigned int mem = 0;

	// we crawl through the memory and analyse if the chunks are continuous,
	// assumption: chunk list sorted ascending by ptr
	while (f || a) {
		if (f == NULL) {
			mem += a->size + sizeof(chunk_list_t);
			t = chunk_ptr(a->next);
			if (t) {
				// check if a is continuous
				void *next = (char *)a->ptr + a->size + sizeof(chunk_list_t);
				if (next != t->ptr) {
					throw BBInconsistentMemoryException("non-contiguos allocated memory");
				}
			}
			a = t;
		} else if (a == NULL) {
			mem += f->size + sizeof(chunk_list_t);
			t = chunk_ptr(f->next);
			if (t) {
				// check if f is continuous
				void *next = (char *)f->ptr + f->size + sizeof(chunk_list_t);
				if (next != t->ptr) {
					throw BBInconsistentMemoryException("non-contiguos allocated memory");
				}
			}
			f = t;
		} else if (f->ptr == a->ptr) {
			throw BBInconsistentMemoryException("ptr cannot be free and allocated at the same time");
		} else if (f->ptr < a->ptr) {
			mem += f->size + sizeof(chunk_list_t);
			void *next = (char *)f->ptr + f->size;
			t          = chunk_ptr(f->next);
			if ((next != t) && (next != a)) {
				throw BBInconsistentMemoryException("there are unallocated bytes between chunks (f)");
			}
			f = t;
		} else {
			mem += a->size + sizeof(chunk_list_t);
			void *next = (char *)a->ptr + a->size;
			t          = chunk_ptr(a->next);
			if ((next != t) && (next != f)) {
				throw BBInconsistentMemoryException("there are unallocated bytes between chunks (a)");
			}
			a = t;
		}
	}

	if (mem != memsize_) {
		throw BBInconsistentMemoryException(
		  "unmanaged memory found, managed memory size != total memory size");
	}
}

/** Check if this BB memory manager is the master.
 * @return true if this BB memory manager instance is the master for the BB
 * shared memory segment, false otherwise
 */
bool
BlackBoardMemoryManager::is_master() const
{
	return master_;
}

/** Print out info about free chunks.
 * Prints out a formatted list of free chunks.
 */
void
BlackBoardMemoryManager::print_free_chunks_info() const
{
	list_print_info(shmem_ ? shmem_header_->free_list_head() : free_list_head_);
}

/** Print out info about allocated chunks.
 * Prints out a formatted list of allocated chunks.
 */
void
BlackBoardMemoryManager::print_allocated_chunks_info() const
{
	list_print_info(shmem_ ? shmem_header_->alloc_list_head() : alloc_list_head_);
}

/** Prints out performance info.
 * This will print out information about the number of free and allocated chunks,
 * the maximum free and allocated chunk size and the number of overhanging bytes
 * (see class description about overhanging bytes).
 */
void
BlackBoardMemoryManager::print_performance_info() const
{
	printf("free chunks: %6u, alloc chunks: %6u, max free: %10u, max alloc: %10u, overhang: %10u\n",
	       list_length(shmem_ ? shmem_header_->free_list_head() : free_list_head_),
	       list_length(shmem_ ? shmem_header_->alloc_list_head() : alloc_list_head_),
	       max_free_size(),
	       max_allocated_size(),
	       overhang_size());
}

/** Get maximum allocatable memory size.
 * This method gives information about the maximum free chunk size and thus
 * the maximum of memory that can be allocated in one chunk.
 * @return maximum free chunk size
 */
unsigned int
BlackBoardMemoryManager::max_free_size() const
{
	chunk_list_t *m = list_get_biggest(shmem_ ? shmem_header_->free_list_head() : free_list_head_);
	if (m == NULL) {
		return 0;
	} else {
		return m->size;
	}
}

/** Get total free memory.
 * This method gives information about the sum of all free chunk sizes. Note that
 * it is not guaranteed that that much data can be stored in the memory since
 * fragmentation may have occured. To get information about the biggest piece
 * of memory that you can allocate use getMaxFreeSize()
 * @return sum of free chunk sizes
 */
unsigned int
BlackBoardMemoryManager::free_size() const
{
	unsigned int  free_size = 0;
	chunk_list_t *l         = shmem_ ? shmem_header_->free_list_head() : free_list_head_;
	while (l) {
		free_size += l->size;
		l = chunk_ptr(l->next);
	}
	return free_size;
}

/** Get total allocated memory.
 * This method gives information about the sum of all allocated chunk sizes.
 * @return sum of allocated chunks sizes
 */
unsigned int
BlackBoardMemoryManager::allocated_size() const
{
	unsigned int  alloc_size = 0;
	chunk_list_t *l          = shmem_ ? shmem_header_->alloc_list_head() : alloc_list_head_;
	while (l) {
		alloc_size += l->size;
		l = chunk_ptr(l->next);
	}
	return alloc_size;
}

/** Get number of allocated chunks.
 * @return number of allocated memory chunks
 */
unsigned int
BlackBoardMemoryManager::num_allocated_chunks() const
{
	return list_length(shmem_ ? shmem_header_->alloc_list_head() : alloc_list_head_);
}

/** Get number of free chunks.
 * @return number of free memory chunks
 */
unsigned int
BlackBoardMemoryManager::num_free_chunks() const
{
	return list_length(shmem_ ? shmem_header_->free_list_head() : free_list_head_);
}

/** Get size of memory.
 * This does not include memory headers, but only the size of the data segment.
 * @return size of memory.
 */
unsigned int
BlackBoardMemoryManager::memory_size() const
{
	return memsize_;
}

/** Get BlackBoard version.
 * @return BlackBoard version
 */
unsigned int
BlackBoardMemoryManager::version() const
{
	return shmem_ ? shmem_header_->version() : 0;
}

/** Lock memory.
 * Locks the whole memory segment used and managed by the memory manager. Will
 * aquire local mutex lock and global semaphore lock in shared memory segment.
 */
void
BlackBoardMemoryManager::lock()
{
	mutex_->lock();
	if (shmem_)
		shmem_->lock_for_write();
}

/** Try to lock memory.
 * Tries to lock the whole memory segment used and managed by the memory manager. Will
 * aquire local mutex lock and global semaphore lock in shared memory segment.
 * The lock has been successfully aquired if both of these locks could be aquired!
 * @return true, if the lock could be aquired, false otherwise.
 */
bool
BlackBoardMemoryManager::try_lock()
{
	if (mutex_->try_lock()) {
		if (shmem_) {
			if (shmem_->try_lock_for_write()) {
				return true;
			} else {
				mutex_->unlock();
			}
		} else {
			return true;
		}
	}

	return false;
}

/** Unlock memory.
 * Releases the lock hold on the shared memory segment and the local mutex lock.
 */
void
BlackBoardMemoryManager::unlock()
{
	if (shmem_)
		shmem_->unlock();
	mutex_->unlock();
}

/** Get maximum alloced memory size.
 * This method gives information about the maximum allocated chunk size and thus
 * the maximum of memory that has been be allocated in one chunk.
 * @return maximum allocated chunk size
 */
unsigned int
BlackBoardMemoryManager::max_allocated_size() const
{
	chunk_list_t *m = list_get_biggest(shmem_ ? shmem_header_->alloc_list_head() : alloc_list_head_);
	if (m == NULL) {
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
BlackBoardMemoryManager::overhang_size() const
{
	unsigned int  overhang = 0;
	chunk_list_t *a        = shmem_ ? shmem_header_->alloc_list_head() : alloc_list_head_;
	while (a) {
		overhang += a->overhang;
		a = chunk_ptr(a->next);
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
	bool          modified = true;
	chunk_list_t *l;
	chunk_list_t *n; // next

	while (modified) {
		modified = false;
		l        = shmem_ ? shmem_header_->free_list_head() : free_list_head_;
		n        = chunk_ptr(l->next);
		while (l && n) {
			if (((char *)l->ptr + l->size + sizeof(chunk_list_t)) == n->ptr) {
				// re-unite
				l->size += n->size + sizeof(chunk_list_t);
				l->next  = n->next;
				modified = true;
			}
			l = n;
			n = chunk_ptr(l->next);
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
	if (list == NULL)
		throw NullPointerException("BlackBoardMemoryManager::list_remove: list == NULL");
	if (rmel == NULL)
		throw NullPointerException("BlackBoardMemoryManager::list_remove: rmel == NULL");

	chunk_list_t *new_head = list;
	chunk_list_t *l        = list;
	chunk_list_t *p        = NULL;

	while (l) {
		if (l == rmel) {
			// found element, now remove
			if (p) {
				// we have a predecessor
				p->next = l->next;
			} else {
				// new head
				new_head = chunk_ptr(l->next);
			}
			break;
		}
		p = l;
		l = chunk_ptr(l->next);
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
	if (addel == NULL)
		throw NullPointerException("BlackBoardMemoryManager::list_add: addel == NULL");

	chunk_list_t *new_head = list;
	chunk_list_t *l        = list;
	chunk_list_t *p        = NULL;

	while (l) {
		if (addel->ptr < l->ptr) {
			// add it here
			addel->next = chunk_addr(l);
			if (p != NULL) {
				// predecessor needs new successor
				// before: p->next == l
				p->next = chunk_addr(addel);
			} else {
				new_head = addel;
			}
			// used as condition below
			l = addel;
			break;
		} else {
			p = l;
			l = chunk_ptr(l->next);
		}
	}

	// if l is not addel it has not yet been added
	if (l != addel) {
		// p is last element of list and != NULL
		addel->next = NULL;
		if (p) {
			p->next = chunk_addr(addel);
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
	while (l) {
		if (l->ptr == ptr) {
			// found it
			return l;
		} else {
			l = chunk_ptr(l->next);
		}
	}
	return NULL;
}

/** Print info about chunks in list.
 * Will print information about chunks in list to stdout. Will give pointer as hexadezimal
 * number, size and overhanging bytes of chunk
 * @param list list with chunks to print
 */
void
BlackBoardMemoryManager::list_print_info(const chunk_list_t *list) const
{
	chunk_list_t *l = (chunk_list_t *)list;
	unsigned int  i = 0;

	while (l) {
		printf("Chunk %3u:  0x%x   size=%10u bytes   overhang=%10u bytes\n",
		       ++i,
		       (unsigned int)(size_t)l->ptr,
		       l->size,
		       l->overhang);
		l = chunk_ptr(l->next);
	}
}

/** Get length of list.
 * @param list list to count
 * @return length of list
 */
unsigned int
BlackBoardMemoryManager::list_length(const chunk_list_t *list) const
{
	unsigned int l = 0;
	while (list) {
		++l;
		list = chunk_ptr(list->next);
	}
	return l;
}

/** Get biggest chunk from list.
 * @param list list to search
 * @return biggest chunk in list
 */
chunk_list_t *
BlackBoardMemoryManager::list_get_biggest(const chunk_list_t *list) const
{
	chunk_list_t *b = (chunk_list_t *)list;
	chunk_list_t *l = (chunk_list_t *)list;
	while (l) {
		if (l->size > b->size) {
			b = l;
		}
		l = chunk_ptr(l->next);
	}

	return b;
}

/** Get first element for chunk iteration.
 * @return Iterator pointing to first memory chunk
 */
BlackBoardMemoryManager::ChunkIterator
BlackBoardMemoryManager::begin()
{
	if (shmem_) {
		return BlackBoardMemoryManager::ChunkIterator(shmem_, shmem_header_->alloc_list_head());
	} else {
		return BlackBoardMemoryManager::ChunkIterator(alloc_list_head_);
	}
}

/** Get end of chunk list.
 * This returns an iterator that points to the element just beyond the allocated
 * chunk list.
 * @return ChunkIterator pointing to a non-existant element beyond the chunk list
 */
BlackBoardMemoryManager::ChunkIterator
BlackBoardMemoryManager::end()
{
	return BlackBoardMemoryManager::ChunkIterator();
}

/** @class BlackBoardMemoryManager::ChunkIterator <blackboard/internal/memory_manager.h>
 * Iterator for memory chunks.
 * The ChunkIterator can be used to iterate over all allocated memory chunks
 * in the memory segment.
 */

/** Constructor.
 * Will create a instance pointing beyond the end of the lits.
 */
BlackBoardMemoryManager::ChunkIterator::ChunkIterator()
{
	shmem_ = NULL;
	cur_   = NULL;
}

/** Constructor
 * @param shmem shared memory segkent
 * @param cur Current element for chunk list
 */
BlackBoardMemoryManager::ChunkIterator::ChunkIterator(SharedMemory *shmem, chunk_list_t *cur)
{
	shmem_ = shmem;
	cur_   = cur;
}

/** Constructor
 * @param cur Current element for chunk list
 */
BlackBoardMemoryManager::ChunkIterator::ChunkIterator(chunk_list_t *cur)
{
	shmem_ = NULL;
	cur_   = cur;
}

/** Copy constructor.
 * @param it Iterator to copy
 */
BlackBoardMemoryManager::ChunkIterator::ChunkIterator(const ChunkIterator &it)
{
	shmem_ = it.shmem_;
	cur_   = it.cur_;
}

/** Increment iterator.
 * Advances to the next element. This is the infix-operator. It may be used
 * like this:
 * @code
 * for (ChunkIterator cit = memmgr->begin(); cit != memmgr->end(); ++cit) {
 *   // your code here
 * }
 * @endcode
 * @return Reference to instance itself after advancing to the next element.
 */
BlackBoardMemoryManager::ChunkIterator &
BlackBoardMemoryManager::ChunkIterator::operator++()
{
	if (cur_ != NULL)
		cur_ = chunk_ptr(cur_->next);

	return *this;
}

/** Increment iterator.
 * Advances to the next element in allocated chunk list. This is the postfix-operator.
 * It may be used like this:
 * @code
 * for (ChunkIterator cit = memmgr->begin(); cit != memmgr->end(); cit++) {
 *   // your code here
 * }
 * @endcode
 * Note that since a copy of the original iterator has to be created an returned it
 * the postfix operation takes both, more CPU time and more memory. If possible (especially
 * if used in a for loop like the example) use the prefix operator!
 * @see operator++()
 * @param inc ignored
 * @return copy of the current instance before advancing to the next element.
 */
BlackBoardMemoryManager::ChunkIterator
BlackBoardMemoryManager::ChunkIterator::operator++(int inc)
{
	ChunkIterator rv(*this);
	if (cur_ != NULL)
		cur_ = chunk_ptr(cur_->next);

	return rv;
}

/** Advance by a certain amount.
 * Can be used to add an integer to the iterator to advance many steps in one go.
 * This operation takes linear time depending on i.
 * @param i steps to advance in list. If i is bigger than the number of remaining
 * elements in the list will stop beyond list.
 * @return reference to current instance after advancing i steps or after reaching
 * end of list.
 */
BlackBoardMemoryManager::ChunkIterator &
BlackBoardMemoryManager::ChunkIterator::operator+(unsigned int i)
{
	for (unsigned int j = 0; (cur_ != NULL) && (j < i); ++j) {
		if (cur_ != NULL)
			cur_ = chunk_ptr(cur_->next);
	}
	return *this;
}

/** Advance by a certain amount.
 * Works like operator+(unsigned int i), provided for convenience.
 * @param i steps to advance in list
 * @return reference to current instance after advancing i steps or after reaching
 * end of list.
 */
BlackBoardMemoryManager::ChunkIterator &
BlackBoardMemoryManager::ChunkIterator::operator+=(unsigned int i)
{
	for (unsigned int j = 0; (cur_ != NULL) && (j < i); ++j) {
		if (cur_ != NULL)
			cur_ = chunk_ptr(cur_->next);
	}
	return *this;
}

/** Check equality of two iterators.
 * Can be used to determine if two iterators point to the same chunk.
 * @param c iterator to compare current instance to
 * @return true, if iterators point to the same chunk, false otherwise
 */
bool
BlackBoardMemoryManager::ChunkIterator::operator==(const ChunkIterator &c) const
{
	return (cur_ == c.cur_);
}

/** Check inequality of two iterators.
 * Can be used to determine if two iterators point to different chunks.
 * @param c iterator to compare current instance to
 * @return true, if iterators point to different chunks of memory, false otherwise
 */
bool
BlackBoardMemoryManager::ChunkIterator::operator!=(const ChunkIterator &c) const
{
	return (cur_ != c.cur_);
}

/** Get memory pointer of chunk.
 * Use this operator to get the pointer to the chunk of memory that this iterator
 * points to.
 * @return pointer to memory
 */
void *BlackBoardMemoryManager::ChunkIterator::operator*() const
{
	if (cur_ == NULL)
		return NULL;

	if (shmem_)
		return shmem_->ptr(cur_->ptr);
	else
		return cur_->ptr;
}

/** Assign iterator.
 * Makes the current instance to point to the same memory element as c.
 * @param c assign value
 * @return reference to current instance
 */
BlackBoardMemoryManager::ChunkIterator &
BlackBoardMemoryManager::ChunkIterator::operator=(const ChunkIterator &c)
{
	shmem_ = c.shmem_;
	cur_   = c.cur_;
	return *this;
}

/** Get size of data segment.
 * Returns the size of the memory chunk. This includes overhanging bytes.
 * @return size of chunk including overhanging bytes
 */
unsigned int
BlackBoardMemoryManager::ChunkIterator::size() const
{
	return (cur_ != NULL) ? cur_->size : 0;
}

/** Get number of overhanging bytes.
 * See documentation of BlackBoardMemoryManager about overhanging bytes.
 * @see BlackBoardMemoryManager
 * @return number of overhanging bytes.
 */
unsigned int
BlackBoardMemoryManager::ChunkIterator::overhang() const
{
	return (cur_ != NULL) ? cur_->overhang : 0;
}

} // end namespace fawkes
