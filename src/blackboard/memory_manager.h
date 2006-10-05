
/***************************************************************************
 *  memory_manager.h - BlackBoard memory manager
 *
 *  Generated: Sat Sep 23 16:00:56 2006 (INSITE 2006, Joburg, South Africa)
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

#ifndef __BLACKBOARD_MEMORY_MANAGER_H_
#define __BLACKBOARD_MEMORY_MANAGER_H_

class BlackBoardSharedMemoryHeader;
class SharedMemory;
class Mutex;

// define our own list type std::list is way too fat
/** Chunk lists as stored in BlackBoard shared memory segment.
 */
typedef struct chunk_list_t {
  chunk_list_t  *next;		/**< pointer to next element in list */
  void          *ptr;		/**< pointer to data segment of chunk */
  unsigned int   size;		/**< total size of chunk, including overhanging bytes,
				 * excluding header */
  unsigned int   overhang;	/**< number of overhanging bytes in this chunk */
};


class BlackBoardMemoryManager
{

 public:
  BlackBoardMemoryManager(unsigned int memsize, unsigned int version,
			  const char *shmem_token = "FawkesBlackBoard");
  ~BlackBoardMemoryManager();


  void * alloc(unsigned int num_bytes);
  void   free(void *chunk_ptr);

  void   check();

  unsigned int getMaxFreeSize();
  unsigned int getMaxAllocatedSize();
  unsigned int getOverhangSize();

  void   printFreeChunksInfo();
  void   printAllocatedChunksInfo();
  void   printPerformanceInfo();

 private:
  chunk_list_t * list_add(chunk_list_t *list, chunk_list_t *addel);
  chunk_list_t * list_remove(chunk_list_t *list, chunk_list_t *rmel);
  chunk_list_t * list_find_ptr(chunk_list_t *list, void *ptr);
  unsigned int   list_length(chunk_list_t *list);
  chunk_list_t * list_get_biggest(chunk_list_t *list);

  void cleanup_free_chunks();

  void list_print_info(chunk_list_t *list);

 private:
  BlackBoardSharedMemoryHeader *shmem_header;
  SharedMemory *shmem;

  unsigned int memsize;

  // Mutex to be used for all list operations (alloc, free)
  Mutex        *mutex;

};


#endif
