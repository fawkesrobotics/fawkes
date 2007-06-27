
/***************************************************************************
 *  shm.h - shared memory segment
 *
 *  Generated: Thu Jan 12 13:12:24 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_IPC_SHM_H_
#define __UTILS_IPC_SHM_H_

// for size_t
#include <sys/types.h>

class SharedMemoryHeader {
 public:
  virtual ~SharedMemoryHeader() {}
  virtual bool         matches(void *memptr)                  = 0;
  virtual size_t       size()                                 = 0;
  virtual void         initialize(void *memptr)               = 0;
  virtual void         set(void *memptr)                      = 0;
  virtual size_t       data_size()                            = 0;
};

class SharedMemoryLister;
class SemaphoreSet;

class SharedMemory
{

 public:

  static const unsigned int MagicTokenSize;

  SharedMemory(const char *magic_token,
	       SharedMemoryHeader *header,
	       bool is_read_only, bool create,
	       bool destroy_on_delete);

  virtual ~SharedMemory();

  bool                is_read_only();
  bool                is_destroyed();
  bool                is_swapable();
  bool                is_valid();
  bool                is_creator();
  bool                is_protected();
  void *              memptr();
  size_t              data_size();
  void                set(void *memptr);
  void                set_destroy_on_delete(bool destroy);
  void                add_semaphore();
  void                set_swapable(bool swapable);

  void                lock();
  bool                try_lock();
  void                unlock();

  void *              ptr(void *addr);
  void *              addr(void *ptr);

  static void         list(char *magic_token,
			   SharedMemoryHeader *header, SharedMemoryLister *lister);

  static void         erase(char *magic_token,
			    SharedMemoryHeader *header, SharedMemoryLister *lister = 0);

  static void         erase_orphaned(char *magic_token,
				     SharedMemoryHeader *header,
				     SharedMemoryLister *lister = 0);

  static bool         exists(char *magic_token,
			     SharedMemoryHeader *header);

  static bool         is_destroyed(int shm_id);
  static bool         is_swapable(int shm_id);
  static unsigned int num_attached(int shm_id);

 protected:

  /** General header.
   * This header is stored right after the magic token.
   */
  typedef struct {
    void        *shm_addr;     /**< Desired shared memory address */
    int          semaphore;    /**< Semaphore set ID */
  } SharedMemory_header_t;

  SharedMemory(char *magic_token,
	       bool is_read_only, bool create, bool destroy_on_delete);

  void attach();
  void free();

  void                   *_memptr;
  size_t                  _mem_size;
  size_t                  _data_size;
  SharedMemoryHeader     *_header;
  bool                    _is_read_only;
  bool                    _destroy_on_delete;
  bool                    _should_create;
  char                   *_magic_token;
  char                   *_shm_magic_token;
  SharedMemory_header_t  *_shm_header;
  void                   *_shm_upper_bound;
  long unsigned int       _shm_offset;


 private:
  void          *__shared_mem;
  int            __shared_mem_id;
  void          *__shared_mem_upper_bound;

  bool           __created;
  SemaphoreSet  *__semset;

};


#endif
