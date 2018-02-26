
/***************************************************************************
 *  shm_lut.cpp - shared memory lookup table
 *
 *  Generated: Thu feb 09 17:32:31 2006
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

#include <fvutils/ipc/shm_lut.h>
#include <fvutils/ipc/shm_exceptions.h>
#include <utils/system/console_colors.h>

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SharedMemoryLookupTable <fvutils/ipc/shm_lut.h>
 * Shared memory lookup table.
 */

/** Write Constructor.
 * Create a new shared memory segment. Will open a shared memory segment that
 * exactly fits the given information. Will throw an error if image with num
 * image_num exists it will throw an exception an exception.
 * I will create a new segment if no matching segment was found.
 * The segment is accessed in read-write mode.
 *
 * @param lut_id LUT ID
 * @param width LUT width
 * @param height LUT height
 * @param depth LUT depth
 * @param bytes_per_cell LUT bytes per cell
 */
SharedMemoryLookupTable::SharedMemoryLookupTable(const char *lut_id,
						 unsigned int width,
						 unsigned int height,
						 unsigned int depth,
						 unsigned int bytes_per_cell)
  : SharedMemory(FIREVISION_SHM_LUT_MAGIC_TOKEN, false, true, true)
{
  constructor(lut_id, width, height, depth, bytes_per_cell, false);
}

/** Read constructor.
 * This constructor is used to search for an existing shared memory segment.
 * It will throw an error if it cannot find a segment with the specified data.
 * The segment is opened read-only by default, but this can be overridden with
 * the is_read_only argument if needed.
 *
 * @param lut_id LUT ID
 * @param is_read_only true to open read-only
 */
SharedMemoryLookupTable::SharedMemoryLookupTable(const char *lut_id,
						 bool is_read_only)
  : SharedMemory(FIREVISION_SHM_LUT_MAGIC_TOKEN, is_read_only, false, false)
{
  constructor(lut_id, 0, 0, 0, 0, is_read_only);
}


void
SharedMemoryLookupTable::constructor(const char *lut_id,
				     unsigned int width, unsigned int height,
				     unsigned int depth,
				     unsigned int bytes_per_cell,
				     bool is_read_only)
{
  _is_read_only    = is_read_only;
  __lut_id         = strdup(lut_id);
  __width          = width;
  __height         = height;
  __depth          = depth;
  __bytes_per_cell = bytes_per_cell;

  __priv_header = new SharedMemoryLookupTableHeader(__lut_id, __width, __height, __depth, __bytes_per_cell);
  _header = __priv_header;
  attach();
  __raw_header = __priv_header->raw_header();

  if (_memptr == NULL) {
    throw Exception("Could not create shared memory segment");
  }
}


/** Destructor. */
SharedMemoryLookupTable::~SharedMemoryLookupTable()
{
  delete __priv_header;
  ::free(__lut_id);
}


/** Get LUT ID.
 * @return LUT ID
 */
const char *
SharedMemoryLookupTable::lut_id() const
{
  return __lut_id;
}


/** Set LUT ID.
 * @param lut_id LUT ID
 * @return true on success
 */
bool
SharedMemoryLookupTable::set_lut_id(const char *lut_id)
{
  free();
  ::free(__lut_id);
  __lut_id = strdup(lut_id);
  __priv_header->set_lut_id(__lut_id);
  attach();
  return (_memptr != NULL);
}


/** Get LUT buffer.
 * @return LUT buffer
 */
unsigned char *
SharedMemoryLookupTable::buffer() const
{
  return (unsigned char *)_memptr;
}


/** Get LUT width.
 * @return LUT width
 */
unsigned int
SharedMemoryLookupTable::width() const
{
  return __raw_header->width;
}


/** Get LUT height.
 * @return LUT height
 */
unsigned int
SharedMemoryLookupTable::height() const
{
  return __raw_header->height;
}


/** Get LUT depth.
 * @return LUT depth
 */
unsigned int
SharedMemoryLookupTable::depth() const
{
  return __raw_header->depth;
}


/** Get bytes per cell.
 * @return bytes per cell
 */
unsigned int
SharedMemoryLookupTable::bytes_per_cell() const
{
  return __raw_header->bytes_per_cell;
}


/** List shared memory LUT segments. */
void
SharedMemoryLookupTable::list()
{
  SharedMemoryLookupTableLister *lister = new SharedMemoryLookupTableLister();
  SharedMemoryLookupTableHeader *h      = new SharedMemoryLookupTableHeader();

  SharedMemory::list(FIREVISION_SHM_LUT_MAGIC_TOKEN, h, lister);

  delete lister;
  delete h;
}


/** Erase all shared memory segments that contain FireVision LUTs.
 * @param use_lister if true a lister is used to print the shared memory segments
 * to stdout while cleaning up.
 */
void
SharedMemoryLookupTable::cleanup(bool use_lister)
{
  SharedMemoryLookupTableLister *lister = NULL;
  SharedMemoryLookupTableHeader *h      = new SharedMemoryLookupTableHeader();

  if ( use_lister ) {
    lister = new SharedMemoryLookupTableLister();
  }

  SharedMemory::erase_orphaned(FIREVISION_SHM_LUT_MAGIC_TOKEN, h, lister);

  delete lister;
  delete h;
}


/** Check LUT availability.
 * @param lut_id image number to check
 * @return true if shared memory segment with requested LUT exists
 */
bool
SharedMemoryLookupTable::exists(const char *lut_id)
{
  SharedMemoryLookupTableHeader *h = new SharedMemoryLookupTableHeader(lut_id, 0, 0, 0, 0);
  bool ex = SharedMemory::exists(FIREVISION_SHM_LUT_MAGIC_TOKEN, h);
  delete h;
  return ex;
}


/** Erase a specific shared memory segment that contains a LUT.
 * @param lut_id LUT ID
 */
void
SharedMemoryLookupTable::wipe(const char *lut_id)
{
  SharedMemoryLookupTableHeader *h = new SharedMemoryLookupTableHeader(lut_id, 0, 0, 0, 0);
  SharedMemory::erase(FIREVISION_SHM_LUT_MAGIC_TOKEN, h, NULL);
  delete h;
}



/** @class SharedMemoryLookupTableHeader <fvutils/ipc/shm_lut.h>
 * Shared memory lookup table header.
 */

/** Constructor. */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader()
{
  __lut_id = NULL;
  __width = 0;
  __height = 0;
  __depth = 0;
  __bytes_per_cell = 0;
  __header = NULL;
}


/** Constructor.
 * @param lut_id LUT ID
 * @param width LUT width
 * @param height LUT height
 * @param bytes_per_cell bytes per cell
 */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader(const char *lut_id,
							     unsigned int width,
							     unsigned int height,
							     unsigned int bytes_per_cell)
{
  __lut_id = strdup(lut_id);
  __width  = width;
  __height = height;
  __bytes_per_cell = bytes_per_cell;

  __header = NULL;
}


/** Constructor.
 * @param lut_id LUT ID
 * @param width LUT width
 * @param height LUT height
 * @param depth LUT depth
 * @param bytes_per_cell bytes per cell
 */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader(const char *lut_id,
							     unsigned int width,
							     unsigned int height,
							     unsigned int depth,
							     unsigned int bytes_per_cell)
{
  __lut_id = strdup(lut_id);
  __width  = width;
  __height = height;
  __depth  = depth;
  __bytes_per_cell = bytes_per_cell;

  __header = NULL;
}


/** Copy constructor.
 * @param h header to copy data from
 */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader(const SharedMemoryLookupTableHeader *h)
{
  if( h->__lut_id != NULL ) {
    __lut_id = strdup(h->__lut_id);
  } else {
    __lut_id = NULL;
  }
  __width  = h->__width;
  __height = h->__height;
  __depth  = h->__depth;
  __bytes_per_cell = h->__bytes_per_cell;

  __header = NULL;
}


/** Destructor. */
SharedMemoryLookupTableHeader::~SharedMemoryLookupTableHeader()
{
  __header = NULL;
  if ( __lut_id != NULL ) {
    free(__lut_id);
    __lut_id = NULL;
  }
}


SharedMemoryHeader *
SharedMemoryLookupTableHeader::clone() const
{
  return new SharedMemoryLookupTableHeader(this);
}


size_t
SharedMemoryLookupTableHeader::size()
{
  return sizeof(SharedMemoryLookupTable_header_t);
}


size_t
SharedMemoryLookupTableHeader::data_size()
{
  if (__header == NULL) {
    return __width * __height * __depth * __bytes_per_cell;
  } else {
    return __header->width * __header->height * __header->depth * __header->bytes_per_cell;
  }
}


bool
SharedMemoryLookupTableHeader::matches(void *memptr)
{
  SharedMemoryLookupTable_header_t *h = (SharedMemoryLookupTable_header_t *)memptr;

  if (__lut_id == NULL) {
    return true;

  } else if (strncmp(h->lut_id, __lut_id, LUT_ID_MAX_LENGTH) == 0) {

    if ( (__width == 0) ||
	 (__height == 0) ||
	 (__depth == 0) ||
	 (__bytes_per_cell == 0) ||
	 ( (h->width == __width) &&
	   (h->height == __height) &&
	   (h->depth == __depth) &&
	   (h->bytes_per_cell == __bytes_per_cell) )
	 ) {
      return true;
    } else {
      throw InconsistentLUTException("Inconsistent lookup table found in memory (meta)");
    }
  } else {
    return false;
  }

}


/** Print Info. */
void
SharedMemoryLookupTableHeader::print_info()
{
  if (__header == NULL) {
    cout << "No image set" << endl;
    return;
  }
  cout << "SharedMemory Lookup Table Info: " << endl
       << "    LUT ID:         " << __header->lut_id << endl
       << "    dimensions:     " << __header->width << "x" << __header->height << "x" 
       << __header->depth << endl
       << "    bytes per cell: " << __header->bytes_per_cell << endl;
}


/** Check if buffer should be created.
 * @return true, if width, height and bytes per cell are all greater than
 * zero.
 */
bool
SharedMemoryLookupTableHeader::create()
{
  return ( (__width > 0) &&
	   (__height > 0) &&
	   (__depth > 0) &&
	   (__bytes_per_cell > 0) );
}


void
SharedMemoryLookupTableHeader::initialize(void *memptr)
{
  __header = (SharedMemoryLookupTable_header_t *)memptr;
  memset(memptr, 0, sizeof(SharedMemoryLookupTable_header_t));
	 
  strncpy(__header->lut_id, __lut_id, LUT_ID_MAX_LENGTH-1);
  __header->width          = __width;
  __header->height         = __height;
  __header->depth          = __depth;
  __header->bytes_per_cell = __bytes_per_cell;
}


void
SharedMemoryLookupTableHeader::set(void *memptr)
{
  __header = (SharedMemoryLookupTable_header_t *)memptr;
}


void
SharedMemoryLookupTableHeader::reset()
{
  __header = NULL;
}


/** Check for equality of headers.
 * First checks if passed SharedMemoryHeader is an instance of
 * SharedMemoryLookupTableHeader. If not returns false, otherwise it compares
 * LUT ID, width, height, depth and bytes per cell. If all match returns true,
 * false if any of them differs.
 * @param s shared memory header to compare to
 * @return true if the two instances identify the very same shared memory segments,
 * false otherwise
 */
bool
SharedMemoryLookupTableHeader::operator==(const SharedMemoryHeader &s) const
{
  const SharedMemoryLookupTableHeader *h = dynamic_cast<const SharedMemoryLookupTableHeader *>(&s);
  if ( ! h ) {
    return false;
  } else {
    return ( (strncmp(__lut_id, h->__lut_id, LUT_ID_MAX_LENGTH) == 0) &&
	     (__width == h->__width) &&
	     (__height == h->__height) &&
	     (__depth == h->__depth) &&
	     (__bytes_per_cell == h->__bytes_per_cell) );
  }
}


/** Get LUT width.
 * @return LUT width.
 */
unsigned int
SharedMemoryLookupTableHeader::width() const
{
  if (__header == NULL) return 0;
  return __header->width;
}


/** Get LUT height.
 * @return LUT height.
 */
unsigned int
SharedMemoryLookupTableHeader::height() const
{
  if (__header == NULL) return 0;
  return __header->height;
}


/** Get LUT depth.
 * @return LUT depth.
 */
unsigned int
SharedMemoryLookupTableHeader::depth() const
{
  if (__header == NULL) return 0;
  return __header->depth;
}


/** Get bytes per cell.
 * @return bytes per cell.
 */
unsigned int
SharedMemoryLookupTableHeader::bytes_per_cell() const
{
  if (__header == NULL) return 0;
  return __header->bytes_per_cell;
}


/** Get LUT ID.
 * @return LUT Id
 */
const char *
SharedMemoryLookupTableHeader::lut_id() const
{
  if (__header == NULL) return NULL;
  return __header->lut_id;
}


/** Set LUT ID.
 * @param lut_id LUT ID
 */
void
SharedMemoryLookupTableHeader::set_lut_id(const char *lut_id)
{
  if ( __lut_id )  free(__lut_id);
  __lut_id = strdup(lut_id);
}


/** Get raw header.
 * @return raw header.
 */
SharedMemoryLookupTable_header_t *
SharedMemoryLookupTableHeader::raw_header()
{
  return __header;
}

/** @class SharedMemoryLookupTableLister <fvutils/ipc/shm_lut.h>
 * Shared memory lookup table lister.
 */


/** Constructor. */
SharedMemoryLookupTableLister::SharedMemoryLookupTableLister()
{
}


/** Destructor. */
SharedMemoryLookupTableLister::~SharedMemoryLookupTableLister()
{
}


void
SharedMemoryLookupTableLister::print_header()
{
  cout << endl << cgreen << "FireVision Shared Memory Segments - Lookup Tables"
       << cnormal << endl
       << "========================================================================================" << endl
       << cdarkgray;
  printf ("%-23s %-10s %-10s %-10s %-9s %-9s %-9s\n",
          "LUT ID", "ShmID", "Semaphore", "Bytes", "Width", "Height", "State");
  cout << cnormal
       << "----------------------------------------------------------------------------------------" << endl;
}


void
SharedMemoryLookupTableLister::print_footer()
{
}


void
SharedMemoryLookupTableLister::print_no_segments()
{
  cout << "No FireVision shared memory segments containing lookup tables found" << endl;
}




void
SharedMemoryLookupTableLister::print_no_orphaned_segments()
{
  cout << "No orphaned FireVision shared memory segments containing lookup tables found" << endl;
}

void
SharedMemoryLookupTableLister::print_info(const SharedMemoryHeader *header,
					  int shm_id, int semaphore,
					  unsigned int mem_size,
					  const void *memptr)
{

  SharedMemoryLookupTableHeader *h = (SharedMemoryLookupTableHeader *)header;

  printf("%-23s %-10d %-10d %-10u %-9u %-9u %s%s\n",
	 h->lut_id(), shm_id, semaphore, mem_size,
	 h->width(), h->height(),
	 (SharedMemory::is_swapable(shm_id) ? "S" : ""),
	 (SharedMemory::is_destroyed(shm_id) ? "D" : "")
	 );
}

} // end namespace firevision
