
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

#include <fvutils/ipc/shm_exceptions.h>
#include <fvutils/ipc/shm_lut.h>
#include <utils/system/console_colors.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace std;
using namespace fawkes;

namespace firevision {

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
SharedMemoryLookupTable::SharedMemoryLookupTable(const char * lut_id,
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
SharedMemoryLookupTable::SharedMemoryLookupTable(const char *lut_id, bool is_read_only)
: SharedMemory(FIREVISION_SHM_LUT_MAGIC_TOKEN, is_read_only, false, false)
{
	constructor(lut_id, 0, 0, 0, 0, is_read_only);
}

void
SharedMemoryLookupTable::constructor(const char * lut_id,
                                     unsigned int width,
                                     unsigned int height,
                                     unsigned int depth,
                                     unsigned int bytes_per_cell,
                                     bool         is_read_only)
{
	_is_read_only   = is_read_only;
	lut_id_         = strdup(lut_id);
	width_          = width;
	height_         = height;
	depth_          = depth;
	bytes_per_cell_ = bytes_per_cell;

	priv_header_ =
	  new SharedMemoryLookupTableHeader(lut_id_, width_, height_, depth_, bytes_per_cell_);
	_header = priv_header_;
	attach();
	raw_header_ = priv_header_->raw_header();

	if (_memptr == NULL) {
		throw Exception("Could not create shared memory segment");
	}
}

/** Destructor. */
SharedMemoryLookupTable::~SharedMemoryLookupTable()
{
	delete priv_header_;
	::free(lut_id_);
}

/** Get LUT ID.
 * @return LUT ID
 */
const char *
SharedMemoryLookupTable::lut_id() const
{
	return lut_id_;
}

/** Set LUT ID.
 * @param lut_id LUT ID
 * @return true on success
 */
bool
SharedMemoryLookupTable::set_lut_id(const char *lut_id)
{
	free();
	::free(lut_id_);
	lut_id_ = strdup(lut_id);
	priv_header_->set_lut_id(lut_id_);
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
	return raw_header_->width;
}

/** Get LUT height.
 * @return LUT height
 */
unsigned int
SharedMemoryLookupTable::height() const
{
	return raw_header_->height;
}

/** Get LUT depth.
 * @return LUT depth
 */
unsigned int
SharedMemoryLookupTable::depth() const
{
	return raw_header_->depth;
}

/** Get bytes per cell.
 * @return bytes per cell
 */
unsigned int
SharedMemoryLookupTable::bytes_per_cell() const
{
	return raw_header_->bytes_per_cell;
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

	if (use_lister) {
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
	SharedMemoryLookupTableHeader *h  = new SharedMemoryLookupTableHeader(lut_id, 0, 0, 0, 0);
	bool                           ex = SharedMemory::exists(FIREVISION_SHM_LUT_MAGIC_TOKEN, h);
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
	lut_id_         = NULL;
	width_          = 0;
	height_         = 0;
	depth_          = 0;
	bytes_per_cell_ = 0;
	header_         = NULL;
}

/** Constructor.
 * @param lut_id LUT ID
 * @param width LUT width
 * @param height LUT height
 * @param bytes_per_cell bytes per cell
 */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader(const char * lut_id,
                                                             unsigned int width,
                                                             unsigned int height,
                                                             unsigned int bytes_per_cell)
{
	lut_id_         = strdup(lut_id);
	width_          = width;
	height_         = height;
	bytes_per_cell_ = bytes_per_cell;

	header_ = NULL;
}

/** Constructor.
 * @param lut_id LUT ID
 * @param width LUT width
 * @param height LUT height
 * @param depth LUT depth
 * @param bytes_per_cell bytes per cell
 */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader(const char * lut_id,
                                                             unsigned int width,
                                                             unsigned int height,
                                                             unsigned int depth,
                                                             unsigned int bytes_per_cell)
{
	lut_id_         = strdup(lut_id);
	width_          = width;
	height_         = height;
	depth_          = depth;
	bytes_per_cell_ = bytes_per_cell;

	header_ = NULL;
}

/** Copy constructor.
 * @param h header to copy data from
 */
SharedMemoryLookupTableHeader::SharedMemoryLookupTableHeader(const SharedMemoryLookupTableHeader *h)
{
	if (h->lut_id_ != NULL) {
		lut_id_ = strdup(h->lut_id_);
	} else {
		lut_id_ = NULL;
	}
	width_          = h->width_;
	height_         = h->height_;
	depth_          = h->depth_;
	bytes_per_cell_ = h->bytes_per_cell_;

	header_ = NULL;
}

/** Destructor. */
SharedMemoryLookupTableHeader::~SharedMemoryLookupTableHeader()
{
	header_ = NULL;
	if (lut_id_ != NULL) {
		free(lut_id_);
		lut_id_ = NULL;
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
	if (header_ == NULL) {
		return (size_t)width_ * height_ * depth_ * bytes_per_cell_;
	} else {
		return (size_t)header_->width * header_->height * header_->depth * header_->bytes_per_cell;
	}
}

bool
SharedMemoryLookupTableHeader::matches(void *memptr)
{
	SharedMemoryLookupTable_header_t *h = (SharedMemoryLookupTable_header_t *)memptr;

	if (lut_id_ == NULL) {
		return true;

	} else if (strncmp(h->lut_id, lut_id_, LUT_ID_MAX_LENGTH) == 0) {
		if ((width_ == 0) || (height_ == 0) || (depth_ == 0) || (bytes_per_cell_ == 0)
		    || ((h->width == width_) && (h->height == height_) && (h->depth == depth_)
		        && (h->bytes_per_cell == bytes_per_cell_))) {
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
	if (header_ == NULL) {
		cout << "No image set" << endl;
		return;
	}
	cout << "SharedMemory Lookup Table Info: " << endl
	     << "    LUT ID:         " << header_->lut_id << endl
	     << "    dimensions:     " << header_->width << "x" << header_->height << "x"
	     << header_->depth << endl
	     << "    bytes per cell: " << header_->bytes_per_cell << endl;
}

/** Check if buffer should be created.
 * @return true, if width, height and bytes per cell are all greater than
 * zero.
 */
bool
SharedMemoryLookupTableHeader::create()
{
	return ((width_ > 0) && (height_ > 0) && (depth_ > 0) && (bytes_per_cell_ > 0));
}

void
SharedMemoryLookupTableHeader::initialize(void *memptr)
{
	header_ = (SharedMemoryLookupTable_header_t *)memptr;
	memset(memptr, 0, sizeof(SharedMemoryLookupTable_header_t));

	strncpy(header_->lut_id, lut_id_, LUT_ID_MAX_LENGTH - 1);
	header_->width          = width_;
	header_->height         = height_;
	header_->depth          = depth_;
	header_->bytes_per_cell = bytes_per_cell_;
}

void
SharedMemoryLookupTableHeader::set(void *memptr)
{
	header_ = (SharedMemoryLookupTable_header_t *)memptr;
}

void
SharedMemoryLookupTableHeader::reset()
{
	header_ = NULL;
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
	if (!h) {
		return false;
	} else {
		return ((strncmp(lut_id_, h->lut_id_, LUT_ID_MAX_LENGTH) == 0) && (width_ == h->width_)
		        && (height_ == h->height_) && (depth_ == h->depth_)
		        && (bytes_per_cell_ == h->bytes_per_cell_));
	}
}

/** Get LUT width.
 * @return LUT width.
 */
unsigned int
SharedMemoryLookupTableHeader::width() const
{
	if (header_ == NULL)
		return 0;
	return header_->width;
}

/** Get LUT height.
 * @return LUT height.
 */
unsigned int
SharedMemoryLookupTableHeader::height() const
{
	if (header_ == NULL)
		return 0;
	return header_->height;
}

/** Get LUT depth.
 * @return LUT depth.
 */
unsigned int
SharedMemoryLookupTableHeader::depth() const
{
	if (header_ == NULL)
		return 0;
	return header_->depth;
}

/** Get bytes per cell.
 * @return bytes per cell.
 */
unsigned int
SharedMemoryLookupTableHeader::bytes_per_cell() const
{
	if (header_ == NULL)
		return 0;
	return header_->bytes_per_cell;
}

/** Get LUT ID.
 * @return LUT Id
 */
const char *
SharedMemoryLookupTableHeader::lut_id() const
{
	if (header_ == NULL)
		return NULL;
	return header_->lut_id;
}

/** Set LUT ID.
 * @param lut_id LUT ID
 */
void
SharedMemoryLookupTableHeader::set_lut_id(const char *lut_id)
{
	if (lut_id_)
		free(lut_id_);
	lut_id_ = strdup(lut_id);
}

/** Get raw header.
 * @return raw header.
 */
SharedMemoryLookupTable_header_t *
SharedMemoryLookupTableHeader::raw_header()
{
	return header_;
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
	cout << endl
	     << cgreen << "FireVision Shared Memory Segments - Lookup Tables" << cnormal << endl
	     << "========================================================================================"
	     << endl
	     << cdarkgray;
	printf("%-23s %-10s %-10s %-10s %-9s %-9s %-9s\n",
	       "LUT ID",
	       "ShmID",
	       "Semaphore",
	       "Bytes",
	       "Width",
	       "Height",
	       "State");
	cout << cnormal
	     << "----------------------------------------------------------------------------------------"
	     << endl;
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
                                          int                       shm_id,
                                          int                       semaphore,
                                          unsigned int              mem_size,
                                          const void *              memptr)
{
	SharedMemoryLookupTableHeader *h = (SharedMemoryLookupTableHeader *)header;

	printf("%-23s %-10d %-10d %-10u %-9u %-9u %s%s\n",
	       h->lut_id(),
	       shm_id,
	       semaphore,
	       mem_size,
	       h->width(),
	       h->height(),
	       (SharedMemory::is_swapable(shm_id) ? "S" : ""),
	       (SharedMemory::is_destroyed(shm_id) ? "D" : ""));
}

} // end namespace firevision
