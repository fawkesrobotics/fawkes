
/***************************************************************************
 *  memory_manager.h - BlackBoard memory manager
 *
 *  Generated: Sat Sep 23 16:00:56 2006 (INSITE 2006, Joburg, South Africa)
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __BLACKBOARD_MEMORY_MANAGER_H_
#define __BLACKBOARD_MEMORY_MANAGER_H_

namespace fawkes {

class BlackBoardSharedMemoryHeader;
class SharedMemory;
class Mutex;
class SemaphoreSet;

// define our own list type std::list is way too fat
/** Chunk lists as stored in BlackBoard shared memory segment.
 * The data segment of a chunk follows directly after the header. So if c is a chunk_list_t
 * pointer to a chunk then the data segment of that chunk can be accessed via
 * (char *)c + sizeof(chunk_list_t).
 */
struct chunk_list_t {
  chunk_list_t  *next;		/**< offset to next element in list */
  void          *ptr;		/**< pointer to data memory */
  unsigned int   size;		/**< total size of chunk, including overhanging bytes,
				 * excluding header */
  unsigned int   overhang;	/**< number of overhanging bytes in this chunk */
};

// May be added later if we want/need per chunk semaphores
//  int            semset_key;	/* key of semaphore for this chunk */
//  unsigned int   reserved   :16;/* reserved bytes */
//  unsigned int   semset_sem : 8;/* semaphore number in semaphore set */


class BlackBoardMemoryManager
{
 friend class BlackBoardInterfaceManager;
 public:
  BlackBoardMemoryManager(unsigned int memsize, unsigned int version, bool master,
			  const char *shmem_token = "FawkesBlackBoard");
  ~BlackBoardMemoryManager();

  void * alloc(unsigned int num_bytes);
  void   free(void *chunk_ptr);

  void   check();

  bool   is_master() const;

  unsigned int max_free_size() const;
  unsigned int max_allocated_size() const;

  unsigned int free_size() const;
  unsigned int allocated_size() const;
  unsigned int overhang_size() const;

  unsigned int num_free_chunks() const;
  unsigned int num_allocated_chunks() const;

  unsigned int memory_size() const;
  unsigned int version() const;

  void   print_free_chunks_info() const;
  void   print_allocated_chunks_info() const;
  void   print_performance_info() const;

  void   lock();
  bool   try_lock();
  void   unlock();

  /*
  void   lock(void *ptr);
  bool   try_lock(void *ptr);
  void   unlock(void *ptr);
  */

  class ChunkIterator
  {
    friend class BlackBoardMemoryManager;
   private:
    ChunkIterator(SharedMemory *shmem, chunk_list_t *cur);
   public:
    ChunkIterator();
    ChunkIterator(const ChunkIterator &it);
    ChunkIterator & operator++ ();        // prefix
    ChunkIterator   operator++ (int inc); // postfix
    ChunkIterator & operator+  (unsigned int i);
    ChunkIterator & operator+= (unsigned int i);
    bool            operator== (const ChunkIterator & c) const;
    bool            operator!= (const ChunkIterator & c) const;
    void *          operator*  () const;
    ChunkIterator & operator=  (const ChunkIterator & c);

    unsigned int    size() const;
    unsigned int    overhang() const;

   private:
    SharedMemory *shmem;
    chunk_list_t *cur;
  };

  ChunkIterator begin();
  ChunkIterator end();

 private:
  bool __master;

  chunk_list_t * list_add(chunk_list_t *list, chunk_list_t *addel);
  chunk_list_t * list_remove(chunk_list_t *list, chunk_list_t *rmel);
  chunk_list_t * list_find_ptr(chunk_list_t *list, void *ptr);
  unsigned int   list_length(const chunk_list_t *list) const;
  chunk_list_t * list_get_biggest(const chunk_list_t *list) const;
  chunk_list_t * list_next(const chunk_list_t *list) const;

  void cleanup_free_chunks();

  void list_print_info(const chunk_list_t *list) const;

  void * alloc_nolock(unsigned int num_bytes);

 private:
  BlackBoardSharedMemoryHeader *shmem_header;
  SharedMemory *shmem;

  unsigned int memsize;

  // Mutex to be used for all list operations (alloc, free)
  Mutex        *mutex;

};

} // end namespace fawkes

#endif
