
/***************************************************************************
 *  shm.h - shared memory segment
 *
 *  Created: Thu Jan 12 13:12:24 2006
 *  Copyright  2005-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef _UTILS_IPC_SHM_H_
#define _UTILS_IPC_SHM_H_

// for size_t
#include <sys/types.h>
#include <utils/ipc/shm_registry.h>

namespace fawkes {

class SharedMemoryHeader
{
public:
	virtual ~SharedMemoryHeader()
	{
	}
	virtual bool                matches(void *memptr)                         = 0;
	virtual size_t              size()                                        = 0;
	virtual void                initialize(void *memptr)                      = 0;
	virtual void                set(void *memptr)                             = 0;
	virtual void                reset()                                       = 0;
	virtual size_t              data_size()                                   = 0;
	virtual SharedMemoryHeader *clone() const                                 = 0;
	virtual bool                operator==(const SharedMemoryHeader &s) const = 0;
};

class SharedMemoryLister;
class SemaphoreSet;

class SharedMemory
{
public:
	static const unsigned int MagicTokenSize;
	static const short        MaxNumConcurrentReaders;

	SharedMemory(const char *        magic_token,
	             SharedMemoryHeader *header,
	             bool                is_read_only,
	             bool                create,
	             bool                destroy_on_delete,
	             const char *        registry_name = 0);

	SharedMemory(const SharedMemory &s);

	virtual ~SharedMemory();

	SharedMemory &operator=(const SharedMemory &s);

	bool         is_read_only() const;
	bool         is_destroyed() const;
	bool         is_swapable() const;
	bool         is_valid() const;
	bool         is_creator() const;
	bool         is_protected() const;
	void *       memptr() const;
	size_t       data_size() const;
	int          shmem_id() const;
	unsigned int num_attached() const;

	void set(void *memptr);
	void set_destroy_on_delete(bool destroy);
	void add_semaphore();
	void set_swapable(bool swapable);

	void lock_for_read();
	bool try_lock_for_read();
	void lock_for_write();
	bool try_lock_for_write();
	void unlock();

	void *ptr(void *addr) const;
	void *addr(void *ptr) const;

	static void list(const char *        magic_token,
	                 SharedMemoryHeader *header,
	                 SharedMemoryLister *lister,
	                 const char *        registry_name = 0);

	static void erase(const char *        magic_token,
	                  SharedMemoryHeader *header,
	                  SharedMemoryLister *lister        = 0,
	                  const char *        registry_name = 0);

	static void erase_orphaned(const char *        magic_token,
	                           SharedMemoryHeader *header,
	                           SharedMemoryLister *lister        = 0,
	                           const char *        registry_name = 0);

	static bool
	exists(const char *magic_token, SharedMemoryHeader *header, const char *registry_name = 0);

	static bool         is_destroyed(int shm_id);
	static bool         is_swapable(int shm_id);
	static unsigned int num_attached(int shm_id);

	class SharedMemoryIterator
	{
	public:
		SharedMemoryIterator();
		SharedMemoryIterator(const SharedMemoryIterator &shmit);
		SharedMemoryIterator(std::list<SharedMemoryRegistry::SharedMemID> ids,
		                     SharedMemoryHeader *                         header);
		~SharedMemoryIterator();

		SharedMemoryIterator &    operator++();        // prefix
		SharedMemoryIterator      operator++(int inc); // postfix
		SharedMemoryIterator &    operator+(unsigned int i);
		SharedMemoryIterator &    operator+=(unsigned int i);
		bool                      operator==(const SharedMemoryIterator &s) const;
		bool                      operator!=(const SharedMemoryIterator &s) const;
		const SharedMemoryHeader *operator*() const;
		SharedMemoryIterator &    operator=(const SharedMemoryIterator &shmit);

		const char *magic_token() const;
		int         shmid() const;
		int         semaphore() const;
		size_t      segmsize() const;
		size_t      segmnattch() const;
		void *      databuf() const;

	private:
		void attach();
		void reset();

		bool                                                   initialized_;
		std::list<SharedMemoryRegistry::SharedMemID>           ids_;
		std::list<SharedMemoryRegistry::SharedMemID>::iterator id_it_;
		int                                                    cur_shmid_;
		SharedMemoryHeader *                                   header_;
		void *                                                 shm_buf_;
		void *                                                 data_buf_;
		int                                                    semaphore_;
		size_t                                                 segmsize_;
		size_t                                                 segmnattch_;
	};

	static SharedMemoryIterator
	                            find(const char *magic_token, SharedMemoryHeader *header, const char *registry_name = 0);
	static SharedMemoryIterator end();

protected:
	/** General header.
   * This header is stored right after the magic token.
   */
	typedef struct
	{
		void *shm_addr;  /**< Desired shared memory address */
		int   semaphore; /**< Semaphore set ID */
	} SharedMemory_header_t;

	SharedMemory(const char *magic_token,
	             bool        is_read_only,
	             bool        create,
	             bool        destroy_on_delete,
	             const char *registry_name = 0);

	void attach();
	void free();

	void *                 _memptr;
	size_t                 _mem_size;
	size_t                 _data_size;
	SharedMemoryHeader *   _header;
	bool                   _is_read_only;
	bool                   _destroy_on_delete;
	bool                   _should_create;
	char *                 _magic_token;
	char *                 _shm_magic_token;
	SharedMemory_header_t *_shm_header;
	void *                 _shm_upper_bound;
	long unsigned int      _shm_offset;

private:
	SharedMemoryRegistry *shm_registry_;
	char *                registry_name_;

	void *shared_mem_;
	int   shared_mem_id_;
	void *shared_mem_upper_bound_;

	bool          created_;
	SemaphoreSet *semset_;

	bool lock_aquired_;
	bool write_lock_aquired_;
};

} // end namespace fawkes

#endif
