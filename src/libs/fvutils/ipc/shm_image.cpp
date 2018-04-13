
/***************************************************************************
 *  shm_image.cpp - shared memory image buffer
 *
 *  Created: Thu Jan 12 14:10:43 2006
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
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

#include <core/exception.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_exceptions.h>
#include <utils/system/console_colors.h>
#include <utils/ipc/shm_exceptions.h>
#include <utils/misc/strndup.h>

#include <iostream>
#include <memory>
#include <cstring>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class SharedMemoryImageBuffer <fvutils/ipc/shm_image.h>
 * Shared memory image buffer.
 * Write images to or retrieve images from a shared memory segment.
 * @author Tim Niemueller
 */

/** Write Constructor.
 * Create a new shared memory segment. Will open a shared memory segment that
 * exactly fits the given information. Will throw an error if image with id
 * image_id exists.
 * I will create a new segment if no matching segment was found.
 * The segment is accessed in read-write mode.
 *
 * Use this constructor to open a shared memory image buffer for writing.
 * @param image_id image ID to open
 * @param cspace colorspace
 * @param width image width
 * @param height image height
 */
SharedMemoryImageBuffer::SharedMemoryImageBuffer(const char *image_id,
						 colorspace_t cspace,
						 unsigned int width,
						 unsigned int height)
  : SharedMemory(FIREVISION_SHM_IMAGE_MAGIC_TOKEN,
		 /* read-only */ false,
		 /* create */ true,
		 /* destroy on delete */ true)
{
  constructor(image_id, cspace, width, height, false);
  add_semaphore();
}


/** Read Constructor.
 * This constructor is used to search for an existing shared memory segment.
 * It will throw an error if it cannot find a segment with the specified data.
 * The segment is opened read-only by default, but this can be overridden with
 * the is_read_only argument if needed.
 *
 * Use this constructor to open an image for reading.
 * @param image_id Image ID to open
 * @param is_read_only true to open image read-only
 */
SharedMemoryImageBuffer::SharedMemoryImageBuffer(const char *image_id, bool is_read_only)
  : SharedMemory(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, is_read_only, /* create */ false, /* destroy */ false)
{
  constructor(image_id, CS_UNKNOWN, 0, 0, is_read_only);
}


void
SharedMemoryImageBuffer::constructor(const char *image_id, colorspace_t cspace,
				     unsigned int width, unsigned int height,
				     bool is_read_only)
{
  _image_id     = strdup(image_id);
  _is_read_only = is_read_only;

  _colorspace = cspace;
  _width      = width;
  _height     = height;

  priv_header = new SharedMemoryImageBufferHeader(_image_id, _colorspace, width, height);
  _header = priv_header;
  try {
    attach();
    raw_header = priv_header->raw_header();
  } catch (Exception &e) {
    e.append("SharedMemoryImageBuffer: could not attach to '%s'\n", image_id);
    ::free(_image_id);
    _image_id = NULL;
    delete priv_header;
    throw;
  }
}


/** Destructor. */
SharedMemoryImageBuffer::~SharedMemoryImageBuffer()
{
  ::free(_image_id);
  delete priv_header;
}


/** Set image number.
 * This will close the currently opened image and will try to open the new
 * image. This operation should be avoided.
 * @param image_id new image ID
 * @return true on success
 */
bool
SharedMemoryImageBuffer::set_image_id(const char *image_id)
{
  free();
  ::free(_image_id);
  _image_id = strdup(image_id);
  priv_header->set_image_id(_image_id);
  attach();
  raw_header = priv_header->raw_header();
  return (_memptr != NULL);
}


/** Set frame ID.
 * @param frame_id new frame ID
 */
void
SharedMemoryImageBuffer::set_frame_id(const char *frame_id)
{
  priv_header->set_frame_id(frame_id);
  strncpy(raw_header->frame_id, frame_id, FRAME_ID_MAX_LENGTH-1);
}


/** Get Image ID.
 * @return image id
 */
const char *
SharedMemoryImageBuffer::image_id() const
{
  return _image_id;
}


/** Get frame ID.
 * @return frame id
 */
const char *
SharedMemoryImageBuffer::frame_id() const
{
  return priv_header->frame_id();
}


/** Get the time when the image was captured.
 * @param sec upon return contains the seconds part of the time
 * @param usec upon return contains the micro seconds part of the time
 */
void
SharedMemoryImageBuffer::capture_time(long int *sec, long int *usec) const
{
  *sec  = raw_header->capture_time_sec;
  *usec = raw_header->capture_time_usec;
}

/** Get the time when the image was captured.
 * @return capture time
 */
Time
SharedMemoryImageBuffer::capture_time() const
{
  return Time(raw_header->capture_time_sec, raw_header->capture_time_usec);
}


/** Set the capture time.
 * @param time capture time
 */
void
SharedMemoryImageBuffer::set_capture_time(Time *time)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting capture time.");
  }

  const timeval *t = time->get_timeval();
  raw_header->capture_time_sec  = t->tv_sec;
  raw_header->capture_time_usec = t->tv_usec;
}

/** Set the capture time.
 * @param sec seconds part of capture time
 * @param usec microseconds part of capture time
 */
void
SharedMemoryImageBuffer::set_capture_time(long int sec, long int usec)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting capture time.");
  }

  raw_header->capture_time_sec  = sec;
  raw_header->capture_time_usec = usec;
}

/** Get image buffer.
 * @return image buffer.
 */
unsigned char *
SharedMemoryImageBuffer::buffer() const
{
  return (unsigned char *)_memptr;
}


/** Get color space.
 * @return colorspace
 */
colorspace_t
SharedMemoryImageBuffer::colorspace() const
{
  return (colorspace_t)raw_header->colorspace;
}


/** Get image width.
 * @return width
 */
unsigned int
SharedMemoryImageBuffer::width() const
{
  return raw_header->width;
}


/** Get image height.
 * @return image height
 */
unsigned int
SharedMemoryImageBuffer::height() const
{
  return raw_header->height;
}


/** Get ROI X.
 * @return ROI X
 */
unsigned int
SharedMemoryImageBuffer::roi_x() const
{
  return raw_header->roi_x;
}


/** Get ROI Y.
 * @return ROI Y
 */
unsigned int
SharedMemoryImageBuffer::roi_y() const
{
  return raw_header->roi_y;
}


/** Get ROI width.
 * @return ROI width
 */
unsigned int
SharedMemoryImageBuffer::roi_width() const
{
  return raw_header->roi_width;
}


/** Get ROI height.
 * @return ROI height
 */
unsigned int
SharedMemoryImageBuffer::roi_height() const
{
  return raw_header->roi_height;
}


/** Get circle X.
 * @return circle X
 */
int
SharedMemoryImageBuffer::circle_x() const
{
  return raw_header->circle_x;
}


/** Get circle Y.
 * @return circle Y
 */
int
SharedMemoryImageBuffer::circle_y() const
{
  return raw_header->circle_y;
}


/** Get circle radius.
 * @return circle radius
 */
unsigned int
SharedMemoryImageBuffer::circle_radius() const
{
  return raw_header->circle_radius;
}


/** Set ROI X.
 * @param roi_x new ROI X
 */
void
SharedMemoryImageBuffer::set_roi_x(unsigned int roi_x)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting ROI X.");
  }
  raw_header->roi_x = roi_x;
}


/** Set ROI Y.
 * @param roi_y new ROI Y
 */
void
SharedMemoryImageBuffer::set_roi_y(unsigned int roi_y)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting ROI Y.");
  }
  raw_header->roi_y = roi_y;
}


/** Set ROI width.
 * @param roi_w new ROI width
 */
void
SharedMemoryImageBuffer::set_roi_width(unsigned int roi_w)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting ROI width.");
  }
  raw_header->roi_width = roi_w;
}


/** Set ROI height.
 * @param roi_h new ROI height
 */
void
SharedMemoryImageBuffer::set_roi_height(unsigned int roi_h)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting ROI height.");
  }
  raw_header->roi_height = roi_h;
}


/** Set ROI data.
 * @param roi_x new ROI X
 * @param roi_y new ROI Y
 * @param roi_w new ROI width
 * @param roi_h new ROI height
 */
void
SharedMemoryImageBuffer::set_roi(unsigned int roi_x, unsigned int roi_y,
				 unsigned int roi_w, unsigned int roi_h)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting ROI X/Y.");
  }
  raw_header->roi_x = roi_x;
  raw_header->roi_y = roi_y;
  raw_header->roi_width  = roi_w;
  raw_header->roi_height = roi_h;
}


/** Set circle X.
 * @param circle_x new circle X
 */
void
SharedMemoryImageBuffer::set_circle_x(int circle_x)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting circle X.");
  }
  raw_header->circle_x = circle_x;
}


/** Set circle Y.
 * @param circle_y new circle Y
 */
void
SharedMemoryImageBuffer::set_circle_y(int circle_y)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting circle Y.");
  }
  raw_header->circle_y = circle_y;
}


/** Set circle radius.
 * @param circle_radius new circle radius
 */
void
SharedMemoryImageBuffer::set_circle_radius(unsigned int circle_radius)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting circle radius.");
  }
  raw_header->circle_radius = circle_radius;
}


/** Set circle data.
 * @param x circle X
 * @param y circle Y
 * @param r circle radius
 */
void
SharedMemoryImageBuffer::set_circle(int x, int y, unsigned int r)
{
  if (_is_read_only) {
    throw Exception("Buffer is read-only. Not setting circle X/Y/radius.");
  }
  raw_header->circle_x      = x;
  raw_header->circle_y      = y;
  raw_header->circle_radius = r;
}


/** Set circle found.
 * @param found true if circle found
 */
void
SharedMemoryImageBuffer::set_circle_found(bool found)
{
  raw_header->flag_circle_found = (found ? 1 : 0);
}


/** Check if circle was found .
 * @return true if circle was found, false otherwise
 */
bool
SharedMemoryImageBuffer::circle_found() const
{
  return (raw_header->flag_circle_found == 1);
}


/** List all shared memory segments that contain a FireVision image. */
void
SharedMemoryImageBuffer::list()
{
  SharedMemoryImageBufferLister *lister = new SharedMemoryImageBufferLister();
  SharedMemoryImageBufferHeader *h      = new SharedMemoryImageBufferHeader();

  SharedMemory::list(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h, lister);

  delete lister;
  delete h;
}


/** Get meta data about image buffers.
 * @return list of meta data
 */
std::list<SharedMemoryImageBufferMetaData>
SharedMemoryImageBuffer::list_meta_data()
{
#if __cplusplus >= 201103L
  std::unique_ptr<SharedMemoryImageBufferMetaDataCollector>
    lister(new SharedMemoryImageBufferMetaDataCollector());
  std::unique_ptr<SharedMemoryImageBufferHeader>
    h(new SharedMemoryImageBufferHeader());
#else
  std::auto_ptr<SharedMemoryImageBufferMetaDataCollector>
    lister(new SharedMemoryImageBufferMetaDataCollector());
  std::auto_ptr<SharedMemoryImageBufferHeader>
    h(new SharedMemoryImageBufferHeader());
#endif

  SharedMemory::list(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h.get(), lister.get());
  return lister->meta_data();
}


/** Erase all shared memory segments that contain FireVision images.
 * @param use_lister if true a lister is used to print the shared memory segments
 * to stdout while cleaning up.
 */
void
SharedMemoryImageBuffer::cleanup(bool use_lister)
{
  SharedMemoryImageBufferLister *lister = NULL;
  SharedMemoryImageBufferHeader *h      = new SharedMemoryImageBufferHeader();

  if (use_lister) {
    lister = new SharedMemoryImageBufferLister();
  }

  SharedMemory::erase_orphaned(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h, lister);

  delete lister;
  delete h;
}


/** Check image availability.
 * @param image_id image ID to check
 * @return true if shared memory segment with requested image exists
 */
bool
SharedMemoryImageBuffer::exists(const char *image_id)
{
  SharedMemoryImageBufferHeader *h      = new SharedMemoryImageBufferHeader(image_id, CS_UNKNOWN, 0, 0);

  bool ex = SharedMemory::exists(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h);

  delete h;
  return ex;
}


/** Erase a specific shared memory segment that contains an image.
 * @param image_id ID of image to wipe
 */
void
SharedMemoryImageBuffer::wipe(const char *image_id)
{
  SharedMemoryImageBufferHeader *h      = new SharedMemoryImageBufferHeader(image_id, CS_UNKNOWN, 0, 0);

  SharedMemory::erase(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h, NULL);

  delete h;
}


/** @class SharedMemoryImageBufferHeader <fvutils/ipc/shm_image.h>
 * Shared memory image buffer header.
 */

/** Constructor. */
SharedMemoryImageBufferHeader::SharedMemoryImageBufferHeader()
{
  _colorspace = CS_UNKNOWN;
  _image_id = NULL;
  _frame_id = NULL;
  _width = 0;
  _height = 0;
  _header = NULL;
  _orig_image_id = NULL;
  _orig_frame_id = NULL;
}


/** Constructor.
 * @param image_id image id
 * @param colorspace colorspace
 * @param width width
 * @param height height
 */
SharedMemoryImageBufferHeader::SharedMemoryImageBufferHeader(const char *image_id,
							     colorspace_t colorspace,
							     unsigned int width,
							     unsigned int height)
{
  _image_id   = strdup(image_id);
  _colorspace = colorspace;
  _width      = width;
  _height     = height;
  _header     = NULL;
  _frame_id   = NULL;

  _orig_image_id   = NULL;
  _orig_frame_id   = NULL;
  _orig_width      = 0;
  _orig_height     = 0;
  _orig_colorspace = CS_UNKNOWN;
}


/** Copy constructor.
 * @param h shared memory image header to copy
 */
SharedMemoryImageBufferHeader::SharedMemoryImageBufferHeader(const SharedMemoryImageBufferHeader *h)
{
  if ( h->_image_id != NULL ) {
    _image_id   = strdup(h->_image_id);
  } else {
    _image_id = NULL;
  }
  if ( h->_frame_id != NULL ) {
    _frame_id   = strdup(h->_frame_id);
  } else {
    _frame_id = NULL;
  }
  _colorspace = h->_colorspace;
  _width      = h->_width;
  _height     = h->_height;
  _header     = h->_header;

  _orig_image_id   = NULL;
  _orig_frame_id   = NULL;
  _orig_width      = 0;
  _orig_height     = 0;
  _orig_colorspace = CS_UNKNOWN;
}


/** Destructor. */
SharedMemoryImageBufferHeader::~SharedMemoryImageBufferHeader()
{
  if ( _image_id != NULL)  free(_image_id);
  if ( _frame_id != NULL)  free(_frame_id);
  if ( _orig_image_id != NULL)  free(_orig_image_id);
  if ( _orig_frame_id != NULL)  free(_orig_frame_id);
}


size_t
SharedMemoryImageBufferHeader::size()
{
  return sizeof(SharedMemoryImageBuffer_header_t);
}


SharedMemoryHeader *
SharedMemoryImageBufferHeader::clone() const
{
  return new SharedMemoryImageBufferHeader(this);
}


size_t
SharedMemoryImageBufferHeader::data_size()
{
  if (_header == NULL) {
    return colorspace_buffer_size(_colorspace, _width, _height);
  } else {
    return colorspace_buffer_size((colorspace_t)_header->colorspace, _header->width, _header->height);
  }
}


bool
SharedMemoryImageBufferHeader::matches(void *memptr)
{
  SharedMemoryImageBuffer_header_t *h = (SharedMemoryImageBuffer_header_t *)memptr;

  if (_image_id == NULL) {
    return true;

  } else if (strncmp(h->image_id, _image_id, IMAGE_ID_MAX_LENGTH) == 0) {
    if ( (_colorspace == CS_UNKNOWN) ||
	 (((colorspace_t)h->colorspace == _colorspace) &&
	  (h->width == _width) &&
	  (h->height == _height) &&
          (! _frame_id || (strncmp(h->frame_id, _frame_id, FRAME_ID_MAX_LENGTH) == 0))
	  )
	 )
    {
      return true;
    } else {
      throw InconsistentImageException("Inconsistent image found in memory (meta)");
    }
  } else {
    return false;
  }
}

/** Check for equality of headers.
 * First checks if passed SharedMemoryHeader is an instance of
 * SharedMemoryImageBufferHeader. If not returns false, otherwise it compares
 * image ID, colorspace, width, and height. If all match returns true, false
 * if any of them differs.
 * @param s shared memory header to compare to
 * @return true if the two instances identify the very same shared memory segments,
 * false otherwise
 */
bool
SharedMemoryImageBufferHeader::operator==(const SharedMemoryHeader &s) const
{
  const SharedMemoryImageBufferHeader *h = dynamic_cast<const SharedMemoryImageBufferHeader *>(&s);
  if ( ! h ) {
    return false;
  } else {
    return ( (strncmp(_image_id, h->_image_id, IMAGE_ID_MAX_LENGTH) == 0) &&
             (! _frame_id || (strncmp(_frame_id, h->_frame_id, FRAME_ID_MAX_LENGTH) == 0)) &&
	     (_colorspace == h->_colorspace) &&
	     (_width == h->_width) &&
	     (_height == h->_height) );
  }
}

/** Print some info. */
void
SharedMemoryImageBufferHeader::print_info()
{
  if (_image_id == NULL) {
    cout << "No image set" << endl;
    return;
  }
  cout << "SharedMemory Image Info: " << endl;
  printf("    address:  %p\n", _header);
  cout << "    image id:  " << _image_id << endl
       << "    frame id:  " << (_frame_id ? _frame_id : "NOT SET") << endl
       << "    colorspace: " << _colorspace << endl
       << "    dimensions: " << _width << "x" << _height << endl;
  /*
     << "    ROI:        at (" << header->roi_x << "," << header->roi_y
       << ")  dim " << header->roi_width << "x" << header->roi_height << endl
       << "    circle:     " << (header->flag_circle_found ? "" : "not ")
       << "found at (" << header->circle_x << "," << header->circle_y
       << ")  radius " << header->circle_radius << endl
       << "    img ready:  " << (header->flag_image_ready ? "yes" : "no") << endl;
  */
}


/** Create if colorspace, width and height have been supplied.
 * @return true if colorspace has been set, width and height are greater than zero.
 */
bool
SharedMemoryImageBufferHeader::create()
{
  return ( (_colorspace != CS_UNKNOWN) &&
	   (_width > 0) &&
	   (_height > 0) );
}


void
SharedMemoryImageBufferHeader::initialize(void *memptr)
{
  SharedMemoryImageBuffer_header_t *header = (SharedMemoryImageBuffer_header_t *)memptr;
  memset(memptr, 0, sizeof(SharedMemoryImageBuffer_header_t));

  strncpy(header->image_id, _image_id, IMAGE_ID_MAX_LENGTH-1);
  if (_frame_id) {
    strncpy(header->frame_id, _frame_id, FRAME_ID_MAX_LENGTH-1);
  }
  header->colorspace = _colorspace;
  header->width      = _width;
  header->height     = _height;

  _header = header;
}


void
SharedMemoryImageBufferHeader::set(void *memptr)
{
  SharedMemoryImageBuffer_header_t *header = (SharedMemoryImageBuffer_header_t *)memptr;
  if ( NULL != _orig_image_id )  free(_orig_image_id);
  if ( NULL != _image_id ) {
    _orig_image_id = strdup(_image_id);
    free(_image_id);
  } else {
    _orig_image_id = NULL;
  }
  if ( NULL != _orig_frame_id )  free(_orig_frame_id);
  if ( NULL != _frame_id ) {
    _orig_frame_id = strdup(_frame_id);
    free(_frame_id);
  } else {
    _orig_frame_id = NULL;
  }
  _orig_width = _width;
  _orig_height = _height;
  _orig_colorspace = _colorspace;
  _header = header;

  _image_id = strndup(header->image_id, IMAGE_ID_MAX_LENGTH);
  _frame_id = strndup(header->frame_id, FRAME_ID_MAX_LENGTH);
  _width = header->width;
  _height = header->height;
  _colorspace = (colorspace_t)header->colorspace;
}


void
SharedMemoryImageBufferHeader::reset()
{
  if ( NULL != _image_id ) {
    free(_image_id);
    _image_id = NULL;
  }
  if ( _orig_image_id != NULL ) {
    _image_id = strdup(_orig_image_id);
  }
  if ( NULL != _frame_id ) {
    free(_frame_id);
    _frame_id = NULL;
  }
  if ( _orig_frame_id != NULL ) {
    _frame_id = strdup(_orig_frame_id);
  }
  _width =_orig_width;
  _height =_orig_height;
  _colorspace =_orig_colorspace;
  _header = NULL;
}


/** Get colorspace.
 * @return colorspace
 */
colorspace_t
SharedMemoryImageBufferHeader::colorspace() const
{
  if ( _header)  return (colorspace_t)_header->colorspace;
  else           return _colorspace;
}


/** Get width.
 * @return image width
 */
unsigned int
SharedMemoryImageBufferHeader::width() const
{
  if ( _header)  return _header->width;
  else           return _width;
}


/** Get height.
 * @return image height
 */
unsigned int
SharedMemoryImageBufferHeader::height() const
{
  if ( _header)  return _header->height;
  else           return _height;
}


/** Get image number
 * @return image number
 */
const char *
SharedMemoryImageBufferHeader::image_id() const
{
  return _image_id;
}


/** Get frame ID.
 * @return reference coordinate frame ID.
 */
const char *
SharedMemoryImageBufferHeader::frame_id() const
{
  return _frame_id;
}


/** Set image id
 * @param image_id image ID
 */
void
SharedMemoryImageBufferHeader::set_image_id(const char *image_id)
{
  if ( _image_id != NULL)  ::free(_image_id);
  _image_id = strdup(image_id);
}


/** Set frame ID.
 * @param frame_id frame ID
 */
void
SharedMemoryImageBufferHeader::set_frame_id(const char *frame_id)
{
  if ( _frame_id != NULL)  ::free(_frame_id);
  _frame_id = strdup(frame_id);
}


/** Get raw header.
 * @return raw header.
 */
SharedMemoryImageBuffer_header_t *
SharedMemoryImageBufferHeader::raw_header()
{
  return _header;
}


/** @class SharedMemoryImageBufferLister <fvutils/ipc/shm_image.h>
 * Shared memory image buffer lister.
 */

/** Constructor. */
SharedMemoryImageBufferLister::SharedMemoryImageBufferLister()
{
}


/** Destructor. */
SharedMemoryImageBufferLister::~SharedMemoryImageBufferLister()
{
}


void
SharedMemoryImageBufferLister::print_header()
{
  cout << endl << cgreen << "FireVision Shared Memory Segments - Images" << cnormal << endl
       << "========================================================================================" << endl
       << cdarkgray;
  printf ("%-20s %-20s %-10s %-10s %-9s %-16s %-5s %-5s %s\n",
          "Image ID", "Frame ID", "ShmID", "Semaphore", "Bytes", "Color Space", "Width", "Height",
	  "State");
  cout << cnormal
       << "----------------------------------------------------------------------------------------" << endl;
}


void
SharedMemoryImageBufferLister::print_footer()
{
}


void
SharedMemoryImageBufferLister::print_no_segments()
{
  cout << "No FireVision shared memory segments found" << endl;
}


void
SharedMemoryImageBufferLister::print_no_orphaned_segments()
{
  cout << "No orphaned FireVision shared memory segments found" << endl;
}


void
SharedMemoryImageBufferLister::print_info(const SharedMemoryHeader *header,
					  int shm_id, int semaphore,
					  unsigned int mem_size,
					  const void *memptr)
{

  SharedMemoryImageBufferHeader *h = (SharedMemoryImageBufferHeader *)header;

  const char *colorspace = colorspace_to_string(h->colorspace());

  printf("%-20s %-20s %-10d %-10d %-9u %-16s %-5u %-5u %s%s\n",
	 h->image_id(), h->frame_id(), shm_id, semaphore, mem_size, colorspace,
	 h->width(), h->height(),
	 (SharedMemory::is_swapable(shm_id) ? "S" : ""),
	 (SharedMemory::is_destroyed(shm_id) ? "D" : "")
	 );
}


/** @class SharedMemoryImageBufferMetaData <fvutils/ipc/shm_image.h>
 * Shared memory image buffer meta data container.
 */

/** Constructor. */
SharedMemoryImageBufferMetaData::SharedMemoryImageBufferMetaData()
{
  image_id = frame_id = "";
  colorspace = CS_UNKNOWN;
  width = height = 0;
  mem_size = 0;
  mem_swapable = false;
  mem_destroyed = false;
}

/** Value constructor.
 * @param image_id Image buffer ID
 * @param frame_id Coordinate frame ID
 * @param colorspace Colorspace 
 * @param width Image width
 * @param height Image height
 * @param mem_size Shared memory buffer size
 * @param mem_swapable True if memory might be moved to swap space
 * @param mem_destroyed True if memory has already been marked destroyed
 */
SharedMemoryImageBufferMetaData::SharedMemoryImageBufferMetaData(const char *image_id, const char *frame_id,
								 colorspace_t colorspace,
								 unsigned int width, unsigned int height,
								 size_t mem_size,
								 bool mem_swapable, bool mem_destroyed)
{
  this->image_id = image_id;
  this->frame_id = frame_id;
  this->colorspace = colorspace;
  this->width = width;
  this->height = height;
  this->mem_size = mem_size;
  this->mem_swapable = mem_swapable;
  this->mem_destroyed = mem_destroyed;
}

/** @class SharedMemoryImageBufferMetaDataCollector <fvutils/ipc/shm_image.h>
 * Collect meta data about shared memory segments.
 */

/** Constructor. */
SharedMemoryImageBufferMetaDataCollector::SharedMemoryImageBufferMetaDataCollector()
{
}


/** Destructor. */
SharedMemoryImageBufferMetaDataCollector::~SharedMemoryImageBufferMetaDataCollector()
{
}


void
SharedMemoryImageBufferMetaDataCollector::print_header()
{
}


void
SharedMemoryImageBufferMetaDataCollector::print_footer()
{
}


void
SharedMemoryImageBufferMetaDataCollector::print_no_segments()
{
}


void
SharedMemoryImageBufferMetaDataCollector::print_no_orphaned_segments()
{
}


void
SharedMemoryImageBufferMetaDataCollector::print_info(const SharedMemoryHeader *header,
					  int shm_id, int semaphore,
					  unsigned int mem_size,
					  const void *memptr)
{
  SharedMemoryImageBufferHeader *h = (SharedMemoryImageBufferHeader *)header;

  meta_data_.push_back(SharedMemoryImageBufferMetaData(h->image_id(), h->frame_id(), h->colorspace(),
						       h->height(), h->width(), mem_size,
						       SharedMemory::is_swapable(shm_id),
						       SharedMemory::is_destroyed(shm_id)));
}

} // end namespace firevision
