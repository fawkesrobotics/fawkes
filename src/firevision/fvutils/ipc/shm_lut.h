
/***************************************************************************
 *  shm_lut.h - shared memory lookup table
 *
 *  Generated: Thu Feb 09 16:57:40 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_IPC_SHM_LUT_H_
#define __FIREVISION_FVUTILS_IPC_SHM_LUT_H_

#include <utils/ipc/shm.h>
#include <utils/ipc/shm_lister.h>

/** Shared memory lookup table header struct. */
typedef struct {
  unsigned int  lut_id;		/**< LUT ID */
  unsigned int  width;		/**< LUT width */
  unsigned int  height;		/**< LUT height */
  unsigned int  bytes_per_cell;	/**< Bytes per cell */
} SharedMemoryLookupTable_header_t;


class SharedMemoryLookupTableHeader : public SharedMemoryHeader {
 public:
  SharedMemoryLookupTableHeader();
  SharedMemoryLookupTableHeader(unsigned int lut_id,
				unsigned int width,
				unsigned int height,
				unsigned int bytes_per_cell);
  virtual ~SharedMemoryLookupTableHeader();

  virtual bool         matches(void *memptr);
  virtual size_t       size();
  virtual bool         create();
  virtual void         initialize(void *memptr);
  virtual void         set(void *memptr);
  virtual void         reset();
  virtual size_t       data_size();

  virtual void         print_info();

  unsigned int  getLutID();
  void          setLutID(unsigned int lut_id);
  unsigned int  getWidth();
  unsigned int  getHeight();
  unsigned int  getBytesPerCell();

  SharedMemoryLookupTable_header_t * getRawHeader();

 private:
  SharedMemoryLookupTable_header_t *header;

  unsigned int   lut_id;
  unsigned int   width;
  unsigned int   height;
  unsigned int   bytes_per_cell;
};

class SharedMemoryLookupTableLister : public SharedMemoryLister {
 public:
  SharedMemoryLookupTableLister();
  virtual ~SharedMemoryLookupTableLister();

  virtual void printHeader();
  virtual void printFooter();
  virtual void printNoSegments();
  virtual void printNoOrphanedSegments();
  virtual void printInfo(SharedMemoryHeader *header,
			 int shm_id, int semaphore, unsigned int mem_size,
			 void *memptr);
};


class SharedMemoryLookupTable : public SharedMemory
{

 public:
  SharedMemoryLookupTable( unsigned int lut_id,
			   unsigned int width, unsigned int height,
			   unsigned int bytes_per_cell = 1
			   );
  SharedMemoryLookupTable(unsigned int lut_id , bool is_read_only = true);
  ~SharedMemoryLookupTable();

  bool             setLutID(unsigned int lut_id);
  unsigned char *  getBuffer();
  unsigned int     getWidth();
  unsigned int     getHeight();
  unsigned int     getBytesPerCell();

  static void      list();
  static void      cleanup();
  static bool      exists(unsigned int lut_id);
  static void      wipe(unsigned int lut_id);

 private:
  void constructor(unsigned int lut_id,
		   unsigned int width, unsigned int height,
		   unsigned int bytes_per_cell,
		   bool is_read_only);

  SharedMemoryLookupTableHeader    *priv_header;
  SharedMemoryLookupTable_header_t *raw_header;

  unsigned int   lut_id;
  unsigned int   width;
  unsigned int   height;
  unsigned int   bytes_per_cell;

};


#endif
