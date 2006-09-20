
/***************************************************************************
 *  shm.h - shared memory buffer
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
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __UTILS_IPC_SHM_H_
#define __UTILS_IPC_SHM_H_

class SharedMemoryHeader {
 public:
  virtual ~SharedMemoryHeader() {}
  virtual bool         matches(unsigned char *buffer)         = 0;
  virtual unsigned int size()                                 = 0;
  virtual void         initialize(unsigned char *buffer)      = 0;
  virtual void         set(unsigned char *buffer)             = 0;
  virtual unsigned int dataSize()                             = 0;
};

class SharedMemoryLister;

class SharedMemory
{

 public:

  static const unsigned int MagicTokenSize;

  SharedMemory(char *magic_token,
	       SharedMemoryHeader *header,
	       bool is_read_only = false, bool create = true,
	       bool destroy_on_delete = true);

  virtual ~SharedMemory();

  bool             isReadOnly();
  bool             isDestroyed();
  bool             isLocked();
  bool             isValid();
  unsigned char *  getBuffer();
  unsigned int     getDataSize();
  void             set(unsigned char *buffer);
  void             setDestroyOnDelete(bool destroy);

  static void      list(char *magic_token,
			SharedMemoryHeader *header, SharedMemoryLister *lister);

  static void      erase(char *magic_token,
			 SharedMemoryHeader *header, SharedMemoryLister *lister = 0);

  static bool      exists(char *magic_token,
			  SharedMemoryHeader *header);

  static bool      isDestroyed(int shm_id);
  static bool      isLocked(int shm_id);

 protected:

  /** General header.
   * This header is stored right after the magic token.
   */
  typedef struct {
    int          semaphore;    /**< Semaphore set ID */
  } SharedMemory_header_t;

  SharedMemory(char *magic_token,
	       bool is_read_only, bool create, bool destroy_on_delete);

  void attach();
  void free();

  unsigned char          *buffer;
  unsigned int            mem_size;
  unsigned int            data_size;
  SharedMemoryHeader     *header;
  bool                    is_read_only;
  bool                    destroy_on_delete;
  bool                    should_create;
  char                   *magic_token;
  char                   *shm_magic_token;
  SharedMemory_header_t  *shm_header;


 private:
  void          *shared_mem;
  int            shared_mem_id;

};


#endif
