
/***************************************************************************
 *  shm.h - shared memory segment
 *
 *  Generated: Thu Jan 12 13:12:24 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_IPC_SHM_H_
#define __UTILS_IPC_SHM_H_

// for size_t
#include <sys/types.h>

namespace fawkes {


class SharedMemoryHeader {
 public:
  virtual ~SharedMemoryHeader() {}
  virtual bool         matches(void *memptr)                  = 0;
  virtual size_t       size()                                 = 0;
  virtual void         initialize(void *memptr)               = 0;
  virtual void         set(void *memptr)                      = 0;
  virtual void         reset()                                = 0;
  virtual size_t       data_size()                            = 0;
  virtual SharedMemoryHeader *  clone() const                 = 0;
  virtual bool         operator==(const SharedMemoryHeader &s) const = 0;
};

class SharedMemoryLister;
class SemaphoreSet;

class SharedMemory
{

 public:

  static const unsigned int MagicTokenSize;
  static const short        MaxNumConcurrentReaders;

  SharedMemory(const char *magic_token,
	       SharedMemoryHeader *header,
	       bool is_read_only, bool create,
	       bool destroy_on_delete);

  SharedMemory(const SharedMemory &s);

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

  void                lock_for_read();
  bool                try_lock_for_read();
  void                lock_for_write();
  bool                try_lock_for_write();
  void                unlock();

  void *              ptr(void *addr);
  void *              addr(void *ptr);

  static void         list(const char *magic_token,
			   SharedMemoryHeader *header, SharedMemoryLister *lister);

  static void         erase(const char *magic_token,
			    SharedMemoryHeader *header, SharedMemoryLister *lister = 0);

  static void         erase_orphaned(const char *magic_token,
				     SharedMemoryHeader *header,
				     SharedMemoryLister *lister = 0);

  static bool         exists(const char *magic_token,
			     SharedMemoryHeader *header);

  static bool         is_destroyed(int shm_id);
  static bool         is_swapable(int shm_id);
  static unsigned int num_attached(int shm_id);

  class SharedMemoryIterator
  {
   public:
    SharedMemoryIterator();
    SharedMemoryIterator(const SharedMemoryIterator &shmit);
    SharedMemoryIterator(const char *magic_token, SharedMemoryHeader *header);
    ~SharedMemoryIterator();

    SharedMemoryIterator &      operator++ ();        // prefix
    SharedMemoryIterator        operator++ (int inc); // postfix
    SharedMemoryIterator &      operator+  (unsigned int i);
    SharedMemoryIterator &      operator+= (unsigned int i);
    bool                        operator== (const SharedMemoryIterator & s) const;
    bool                        operator!= (const SharedMemoryIterator & s) const;
    const SharedMemoryHeader *  operator*  () const;
    SharedMemoryIterator &      operator=  (const SharedMemoryIterator & shmit);

    const char *                magic_token() const;
    int                         shmid() const;
    int                         semaphore() const;
    size_t                      segmsize() const;
    size_t                      segmnattch() const;
    void *                      databuf() const;

  private:
    void attach();
    void reset();

    char                   *__magic_token;
    int                     __max_id;
    int                     __cur_shmid;
    int                     __cur_id;
    SharedMemoryHeader     *__header;
    void                   *__shm_buf;
    void                   *__data_buf;
    int                     __semaphore;
    size_t                  __segmsize;
    size_t                  __segmnattch;
  };

  static SharedMemoryIterator find(const char *magic_token, SharedMemoryHeader *header);
  static SharedMemoryIterator end();

 protected:

  /** General header.
   * This header is stored right after the magic token.
   */
  typedef struct {
    void        *shm_addr;     /**< Desired shared memory address */
    int          semaphore;    /**< Semaphore set ID */
  } SharedMemory_header_t;

  SharedMemory(const char *magic_token,
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

  bool           __lock_aquired;
  bool           __write_lock_aquired;

};


} // end namespace fawkes

#endif
