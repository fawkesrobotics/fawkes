
/***************************************************************************
 *  shm_lut.h - shared memory lookup table
 *
 *  Generated: Thu Feb 09 16:57:40 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_IPC_SHM_LUT_H_
#define __FIREVISION_FVUTILS_IPC_SHM_LUT_H_

#include <utils/ipc/shm.h>
#include <utils/ipc/shm_lister.h>
#include <fvutils/ipc/defs.h>
#include <stdint.h>

// Magic token to identify FireVision shared memory LUTs
#define FIREVISION_SHM_LUT_MAGIC_TOKEN "FireVision LUT"

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Shared memory lookup table header struct. */
typedef struct {
  char      lut_id[LUT_ID_MAX_LENGTH];		/**< LUT ID */
  uint32_t  width;		/**< LUT width */
  uint32_t  height;		/**< LUT height */
  uint32_t  depth;              /**< LUT depth */
  uint32_t  bytes_per_cell;	/**< Bytes per cell */
} SharedMemoryLookupTable_header_t;


class SharedMemoryLookupTableHeader : public fawkes::SharedMemoryHeader
{
 public:
  SharedMemoryLookupTableHeader();
  SharedMemoryLookupTableHeader(const char *lut_id,
				unsigned int width,
				unsigned int height,
				unsigned int bytes_per_cell);
  SharedMemoryLookupTableHeader(const char *lut_id,
				unsigned int width,
				unsigned int height,
				unsigned int depth,
				unsigned int bytes_per_cell);
  SharedMemoryLookupTableHeader(const SharedMemoryLookupTableHeader *h);
  virtual ~SharedMemoryLookupTableHeader();

  virtual fawkes::SharedMemoryHeader *  clone() const;
  virtual bool         matches(void *memptr);
  virtual size_t       size();
  virtual bool         create();
  virtual void         initialize(void *memptr);
  virtual void         set(void *memptr);
  virtual void         reset();
  virtual size_t       data_size();
  virtual bool         operator==(const fawkes::SharedMemoryHeader & s) const;

  virtual void         print_info();

  const char *  lut_id() const;
  void          set_lut_id(const char *lut_id);
  unsigned int  width() const;
  unsigned int  height() const;
  unsigned int  depth() const;
  unsigned int  bytes_per_cell() const;

  SharedMemoryLookupTable_header_t * raw_header();

 private:
  SharedMemoryLookupTable_header_t *__header;

  char          *__lut_id;
  unsigned int   __width;
  unsigned int   __height;
  unsigned int   __depth;
  unsigned int   __bytes_per_cell;
};

class SharedMemoryLookupTableLister : public fawkes::SharedMemoryLister
{
 public:
  SharedMemoryLookupTableLister();
  virtual ~SharedMemoryLookupTableLister();

  virtual void print_header();
  virtual void print_footer();
  virtual void print_no_segments();
  virtual void print_no_orphaned_segments();
  virtual void print_info(const fawkes::SharedMemoryHeader *header,
			  int shm_id, int semaphore, unsigned int mem_size,
			  const void *memptr);
};


class SharedMemoryLookupTable : public fawkes::SharedMemory
{

 public:

  SharedMemoryLookupTable( const char *lut_id,
			   unsigned int width, unsigned int height,
			   unsigned int depth = 1,
			   unsigned int bytes_per_cell = 1
			   );
  SharedMemoryLookupTable(const char *lut_id , bool is_read_only = true);
  ~SharedMemoryLookupTable();

  const char *     lut_id() const;
  bool             set_lut_id(const char *lut_id);
  unsigned char *  buffer() const;
  unsigned int     width() const;
  unsigned int     height() const;
  unsigned int     depth() const;
  unsigned int     bytes_per_cell() const;

  static void      list();
  static void      cleanup(bool use_lister = true);
  static bool      exists(const char *lut_id);
  static void      wipe(const char *lut_id);

 private:
  void constructor(const char *lut_id,
		   unsigned int width, unsigned int height, unsigned int depth,
		   unsigned int bytes_per_cell,
		   bool is_read_only);

  SharedMemoryLookupTableHeader    *__priv_header;
  SharedMemoryLookupTable_header_t *__raw_header;

  char          *__lut_id;
  unsigned int   __width;
  unsigned int   __height;
  unsigned int   __depth;
  unsigned int   __bytes_per_cell;

};

} // end namespace firevision

#endif
