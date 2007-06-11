
/***************************************************************************
 *  shm_lut.cpp - shared memory lookup table
 *
 *  Generated: Thu feb 09 17:32:31 2006
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

#include <fvutils/ipc/shm_lut.h>
#include <fvutils/ipc/shm_registry.h>
#include <fvutils/ipc/shm_exceptions.h>
#include <utils/system/console_colors.h>

#include <iostream>

using namespace std;


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
 * @param bytes_per_cell LUT bytes per cell
 */
SharedMemoryLookupTable::SharedMemoryLookupTable(unsigned int lut_id,
						 unsigned int width,
						 unsigned int height,
						 unsigned int bytes_per_cell)
  : SharedMemory(FIREVISION_SHM_LUT_MAGIC_TOKEN, false, true, true)
{
  constructor(lut_id, width, height, bytes_per_cell, false);
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
SharedMemoryLookupTable::SharedMemoryLookupTable(unsigned int lut_id,
						 bool is_read_only)
  : SharedMemory(FIREVISION_SHM_LUT_MAGIC_TOKEN, is_read_only, false, false)
{
  constructor(lut_id, 0, 0, 0, is_read_only);
}


void
SharedMemoryLookupTable::constructor(unsigned int lut_id,
				     unsigned int width, unsigned int height,
				     unsigned int bytes_per_cell,
				     bool is_read_only)
{
  this->lut_id         = lut_id;
  _is_read_only   = is_read_only;
  this->width          = width;
  this->height         = height;
  this->bytes_per_cell = bytes_per_cell;

  priv_header = new SharedMemoryLookupTableHeader(lut_id, width, height, bytes_per_cell);
  _header = priv_header;
  attach();
  raw_header = priv_header->getRawHeader();

  if (_memptr == NULL) {
    throw Exception("Could not create shared memory segment");
  }
}


/** Destructor. */
SharedMemoryLookupTable::~SharedMemoryLookupTable()
{
}


/** Set LUT ID.
 * @param lut_id LUT ID
 * @return true on success
 */
bool
SharedMemoryLookupTable::setLutID(unsigned int lut_id)
{
  free();
  priv_header->setLutID(lut_id);
  attach();
  return (_memptr != NULL);
}


/** Get LUT buffer.
 * @return LUT buffer
 */
unsigned char *
SharedMemoryLookupTable::getBuffer()
{
  return (unsigned char *)_memptr;
}


/** Get LUT width.
 * @return LUT width
 */
unsigned int
SharedMemoryLookupTable::getWidth()
{
  return raw_header->width;
}


/** Get LUT height.
 * @return LUT height
 */
unsigned int
SharedMemoryLookupTable::getHeight()
{
  return raw_header->height;
}


/** Get bytes per cell.
 * @return bytes per cell
 */
unsigned int
SharedMemoryLookupTable::getBytesPerCell()
{
  return raw_header->bytes_per_cell;
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


/** Erase all shared memory segments that contain FireVision LUTs. */
void
SharedMemoryLookupTable::cleanup()
{
  SharedMemoryLookupTableLister *lister = new SharedMemoryLookupTableLister();
  SharedMemoryLookupTableHeader *h      = new SharedMemoryLookupTableHeader();

  SharedMemory::erase(FIREVISION_SHM_LUT_MAGIC_TOKEN, h, lister);

  delete lister;
  delete h;
}


/** Check LUT availability.
 * @param lut_id image number to check
 * @return true if shared memory segment with requested LUT exists
 */
bool
SharedMemoryLookupTable::exists(unsigned int lut_id)
{
  SharedMemoryLookupTableHeader *h      = new SharedMemoryLookupTableHeader(lut_id, 0, 0, 0);

  bool ex = SharedMemory::exists(FIREVISION_SHM_LUT_MAGIC_TOKEN, h);

  delete h;

  return ex;
}


/** Erase a specific shared memory segment that contains a LUT.
 * @param lut_id LUT ID
 */
void
SharedMemoryLookupTable::wipe(unsigned int lut_id)
{
  SharedMemoryLookupTableHeader *h      = new SharedMemoryLookupTableHeader(lut_id, 0, 0, 0);

  SharedMemory::erase(FIREVISION_SHM_LUT_MAGIC_TOKEN, h, NULL);

  delete h;
}



/** @class SharedMemoryLookupTableHeader <fvutils/ipc/shm_lut.h>
 * Shared memory lookup table header.
 */

/** Constructor. */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader()
{
  lut_id = 0xFFFFFFFF;
  width = 0;
  height = 0;
  bytes_per_cell = 0;
  header = NULL;
}


/** Constructor.
 * @param lut_id LUT ID
 * @param width LUT width
 * @param height LUT height
 * @param bytes_per_cell bytes per cell
 */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader(unsigned int lut_id,
							     unsigned int width,
							     unsigned int height,
							     unsigned int bytes_per_cell)
{
  this->lut_id = lut_id;
  this->width  = width;
  this->height = height;
  this->bytes_per_cell = bytes_per_cell;

  header = NULL;
}


/** Destructor. */
SharedMemoryLookupTableHeader::~SharedMemoryLookupTableHeader()
{
  header = NULL;
}


size_t
SharedMemoryLookupTableHeader::size()
{
  return sizeof(SharedMemoryLookupTable_header_t);
}


size_t
SharedMemoryLookupTableHeader::data_size()
{
  if (header == NULL) {
    return width * height * bytes_per_cell;
  } else {
    return header->width * header->height * header->bytes_per_cell;
  }
}


bool
SharedMemoryLookupTableHeader::matches(void *memptr)
{
  SharedMemoryLookupTable_header_t *h = (SharedMemoryLookupTable_header_t *)memptr;

  if (lut_id == 0xFFFFFFFF) {
    return true;

  } else if (h->lut_id == lut_id) {

    if ( (width == 0) ||
	 (height == 0) ||
	 (bytes_per_cell == 0) ||
	 ( (h->width == width) &&
	   (h->height == height) &&
	   (h->bytes_per_cell == bytes_per_cell) )
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
  if (header == NULL) {
    cout << "No image set" << endl;
    return;
  }
  cout << "SharedMemory Lookup Table Info: " << endl
       << "    LUT ID:         " << header->lut_id << endl
       << "    dimensions:     " << header->width << "x" << header->height << endl
       << "    bytes per cell: " << header->bytes_per_cell << endl;
}


/** Check if buffer should be created.
 * @return true, if width, height and bytes per cell are all greater than
 * zero.
 */
bool
SharedMemoryLookupTableHeader::create()
{
  return ( (width > 0) &&
	   (height > 0) &&
	   (bytes_per_cell > 0) );
}


void
SharedMemoryLookupTableHeader::initialize(void *memptr)
{
  header = (SharedMemoryLookupTable_header_t *)memptr;
  memset(memptr, 0, sizeof(SharedMemoryLookupTable_header_t));
	 
  header->lut_id         = lut_id;
  header->width          = width;
  header->height         = height;
  header->bytes_per_cell = bytes_per_cell;
}


void
SharedMemoryLookupTableHeader::set(void *memptr)
{
  header = (SharedMemoryLookupTable_header_t *)memptr;
}


/** Get LUT width.
 * @return LUT width.
 */
unsigned int
SharedMemoryLookupTableHeader::getWidth()
{
  if (header == NULL) return 0;
  return header->width;
}


/** Get LUT height.
 * @return LUT height.
 */
unsigned int
SharedMemoryLookupTableHeader::getHeight()
{
  if (header == NULL) return 0;
  return header->height;
}


/** Get bytes per cell.
 * @return bytes per cell.
 */
unsigned int
SharedMemoryLookupTableHeader::getBytesPerCell()
{
  if (header == NULL) return 0;
  return header->bytes_per_cell;
}


/** Get LUT ID.
 * @return LUT Id
 */
unsigned int
SharedMemoryLookupTableHeader::getLutID()
{
  if (header == NULL) return 0xFFFFFFFF;
  return header->lut_id;
}


/** Set LUT ID.
 * @param lut_id LUT ID
 */
void
SharedMemoryLookupTableHeader::setLutID(unsigned int lut_id)
{
  this->lut_id = lut_id;
}


/** Get raw header.
 * @return raw header.
 */
SharedMemoryLookupTable_header_t *
SharedMemoryLookupTableHeader::getRawHeader()
{
  return header;
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
SharedMemoryLookupTableLister::printHeader()
{
  cout << endl << cgreen << "FireVision Shared Memory Segments - Lookup Tables"
       << cnormal << endl
       << "========================================================================" << endl
       << cwhite;
  printf ("%-3s %-10s %-10s %-10s %-9s %-9s %-9s\n",
          "#", "ShmID", "Semaphore", "Bytes", "Width", "Height", "State");
  cout << cnormal
       << "------------------------------------------------------------------------" << endl;
}


void
SharedMemoryLookupTableLister::printFooter()
{
}


void
SharedMemoryLookupTableLister::printNoSegments()
{
  cout << "No FireVision shared memory segments containing lookup tables found" << endl;
}




void
SharedMemoryLookupTableLister::printNoOrphanedSegments()
{
  cout << "No orphaned FireVision shared memory segments containing lookup tables found" << endl;
}

void
SharedMemoryLookupTableLister::printInfo(SharedMemoryHeader *header,
					 int shm_id, int semaphore,
					 unsigned int mem_size,
					 void *memptr)
{

  SharedMemoryLookupTableHeader *h = (SharedMemoryLookupTableHeader *)header;

  printf("%-3d %-10d %-10d %-10d %-9d %-9d %s%s\n",
	 h->getLutID(), shm_id, semaphore, mem_size,
	 h->getWidth(), h->getHeight(),
	 (SharedMemory::is_swapable(shm_id) ? "S" : ""),
	 (SharedMemory::is_destroyed(shm_id) ? "D" : "")
	 );
}
