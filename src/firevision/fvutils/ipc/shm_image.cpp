
/***************************************************************************
 *  shm_image.cpp - shared memory image buffer
 *
 *  Generated: Thu Jan 12 14:10:43 2006
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

#include <core/exception.h>
#include <fvutils/ipc/shm_image.h>
#include <fvutils/ipc/shm_registry.h>
#include <fvutils/ipc/shm_exceptions.h>
#include <utils/system/console_colors.h>
#include <utils/ipc/shm_exceptions.h>

#include <fvutils/net/fuse.h>

#include <iostream>

using namespace std;

/** @class SharedMemoryImageBuffer <fvutils/ipc/shm_image.h>
 * Shared memory image buffer.
 * Write images to or retrieve images from a shared memory segment.
 * @author Tim Niemueller
 */

/** Write Constructor.
 * Create a new shared memory segment. Will open a shared memory segment that
 * exactly fits the given information. Will throw an error if image with num
 * image_num exists it will throw an exception an exception.
 * I will create a new segment if no matching segment was found.
 * The segment is accessed in read-write mode.
 *
 * Use this constructor to open a shared memory image buffer for writing.
 * @param cspace colorspace
 * @param width image width
 * @param height image height
 * @param image_num image number
 */
SharedMemoryImageBuffer::SharedMemoryImageBuffer(colorspace_t cspace,
						 unsigned int width,
						 unsigned int height,
						 unsigned int image_num)
  : SharedMemory(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, false, true, true)
{
  constructor(cspace, width, height, image_num, false);
}


/** Read Constructor.
 * This constructor is used to search for an existing shared memory segment.
 * It will throw an error if it cannot find a segment with the specified data.
 * The segment is opened read-only by default, but this can be overridden with
 * the is_read_only argument if needed.
 *
 * Use this constructor to open an image for reading.
 * @param image_num image number
 * @param is_read_only true to open image read-only
 */
SharedMemoryImageBuffer::SharedMemoryImageBuffer(unsigned int image_num, bool is_read_only)
  : SharedMemory(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, is_read_only, false, false)
{
  constructor(CS_UNKNOWN, 0, 0, image_num, is_read_only);
}


void
SharedMemoryImageBuffer::constructor(colorspace_t cspace,
				     unsigned int width, unsigned int height,
				     unsigned int image_num,
				     bool is_read_only)
{
  this->image_num = image_num;
  this->is_read_only = is_read_only;

  this->colorspace = cspace;
  this->width      = width;
  this->height     = height;

  priv_header = new SharedMemoryImageBufferHeader(image_num, colorspace, width, height);
  header = priv_header;
  attach();
  raw_header = priv_header->getRawHeader();

  if (memptr == NULL) {
    throw Exception("Could not create shared memory segment");
  }

}


/** Destructor. */
SharedMemoryImageBuffer::~SharedMemoryImageBuffer()
{
}


/** Set image number.
 * @param image_num new image number
 * @return true on success
 */
bool
SharedMemoryImageBuffer::setImageNumber(unsigned int image_num)
{
  free();
  priv_header->setImageNumber(image_num);
  attach();
  raw_header = priv_header->getRawHeader();
  return (memptr != NULL);
}


/** Get image buffer.
 * @return image buffer.
 */
unsigned char *
SharedMemoryImageBuffer::getBuffer()
{
  return (unsigned char *)memptr;
}


/** Get color space.
 * @return colorspace
 */
colorspace_t
SharedMemoryImageBuffer::getColorspace()
{
  return fuse_ui2cs(raw_header->colorspace);
}


/** Get image width.
 * @return width
 */
unsigned int
SharedMemoryImageBuffer::getWidth()
{
  return raw_header->width;
}


/** Get image height.
 * @return image height
 */
unsigned int
SharedMemoryImageBuffer::getHeight()
{
  return raw_header->height;
}


/** Get ROI X.
 * @return ROI X
 */
unsigned int
SharedMemoryImageBuffer::getROIX()
{
  return raw_header->roi_x;
}


/** Get ROI Y.
 * @return ROI Y
 */
unsigned int
SharedMemoryImageBuffer::getROIY()
{
  return raw_header->roi_y;
}


/** Get ROI width.
 * @return ROI width
 */
unsigned int
SharedMemoryImageBuffer::getROIWidth()
{
  return raw_header->roi_width;
}


/** Get ROI height.
 * @return ROI height
 */
unsigned int
SharedMemoryImageBuffer::getROIHeight()
{
  return raw_header->roi_height;
}


/** Get circle X.
 * @return circle X
 */
int
SharedMemoryImageBuffer::getCircleX()
{
  return raw_header->circle_x;
}


/** Get circle Y.
 * @return circle Y
 */
int
SharedMemoryImageBuffer::getCircleY()
{
  return raw_header->circle_y;
}


/** Get circle radius.
 * @return circle radius
 */
unsigned int
SharedMemoryImageBuffer::getCircleRadius()
{
  return raw_header->circle_radius;
}


/** Set ROI X.
 * @param roi_x new ROI X
 */
void
SharedMemoryImageBuffer::setROIX(unsigned int roi_x)
{
  if (is_read_only) {
    throw Exception("Buffer is read-only. Not setting ROI X.");
  }
  raw_header->roi_x = roi_x;
}


/** Set ROI Y.
 * @param roi_y new ROI Y
 */
void
SharedMemoryImageBuffer::setROIY(unsigned int roi_y)
{
  if (is_read_only) {
    throw Exception("Buffer is read-only. Not setting ROI Y.");
  }
  raw_header->roi_y = roi_y;
}


/** Set ROI width.
 * @param roi_w new ROI width
 */
void
SharedMemoryImageBuffer::setROIWidth(unsigned int roi_w)
{
  if (is_read_only) {
    throw Exception("Buffer is read-only. Not setting ROI width.");
  }
  raw_header->roi_width = roi_w;
}


/** Set ROI height.
 * @param roi_h new ROI height
 */
void
SharedMemoryImageBuffer::setROIHeight(unsigned int roi_h)
{
  if (is_read_only) {
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
SharedMemoryImageBuffer::setROI(unsigned int roi_x, unsigned int roi_y,
				unsigned int roi_w, unsigned int roi_h)
{
  if (is_read_only) {
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
SharedMemoryImageBuffer::setCircleX(int circle_x)
{
  if (is_read_only) {
    throw Exception("Buffer is read-only. Not setting circle X.");
  }
  raw_header->circle_x = circle_x;
}


/** Set circle Y.
 * @param circle_y new circle Y
 */
void
SharedMemoryImageBuffer::setCircleY(int circle_y)
{
  if (is_read_only) {
    throw Exception("Buffer is read-only. Not setting circle Y.");
  }
  raw_header->circle_y = circle_y;
}


/** Set circle radius.
 * @param circle_radius new circle radius
 */
void
SharedMemoryImageBuffer::setCircleRadius(unsigned int circle_radius)
{
  if (is_read_only) {
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
SharedMemoryImageBuffer::setCircle(int x, int y, unsigned int r)
{
  if (is_read_only) {
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
SharedMemoryImageBuffer::setCircleFound(bool found)
{
  raw_header->flag_circle_found = (found ? 1 : 0);
}


/** Check if circle was found .
 * @return true if circle was found, false otherwise
 */
bool
SharedMemoryImageBuffer::getCircleFound()
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


/** Erase all shared memory segments that contain FireVision images. */
void
SharedMemoryImageBuffer::cleanup()
{
  SharedMemoryImageBufferLister *lister = new SharedMemoryImageBufferLister();
  SharedMemoryImageBufferHeader *h      = new SharedMemoryImageBufferHeader();

  SharedMemory::erase(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h, lister);

  delete lister;
  delete h;
}


/** Check image availability.
 * @param image_num image number to check
 * @return true if shared memory segment with requested image exists
 */
bool
SharedMemoryImageBuffer::exists(unsigned int image_num)
{
  SharedMemoryImageBufferHeader *h      = new SharedMemoryImageBufferHeader(image_num, CS_UNKNOWN, 0, 0);

  bool ex = SharedMemory::exists(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h);

  delete h;
  return ex;
}


/** Erase a specific shared memory segment that contains an image.
 * @param image_num image number
 */
void
SharedMemoryImageBuffer::wipe(unsigned int image_num)
{
  SharedMemoryImageBufferHeader *h      = new SharedMemoryImageBufferHeader(image_num, CS_UNKNOWN, 0, 0);

  SharedMemory::erase(FIREVISION_SHM_IMAGE_MAGIC_TOKEN, h, NULL);

  delete h;
}


/** @class SharedMemoryImageBufferHeader <fvutils/ipc/shm_image.h>
 * Shared memory image buffer header.
 */

/** Constructor. */
SharedMemoryImageBufferHeader::SharedMemoryImageBufferHeader()
{
  colorspace = CS_UNKNOWN;
  image_num = 0xFFFFFFFF;
  width = 0;
  height = 0;
  header = NULL;
}


/** Constructor.
 * @param image_num image number
 * @param colorspace colorspace
 * @param width width
 * @param height height
 */
SharedMemoryImageBufferHeader::SharedMemoryImageBufferHeader(unsigned int image_num,
							     colorspace_t colorspace,
							     unsigned int width,
							     unsigned int height)
{
  this->image_num  = image_num;
  this->colorspace = colorspace;
  this->width      = width;
  this->height     = height;

  header = NULL;
}


/** Destructor. */
SharedMemoryImageBufferHeader::~SharedMemoryImageBufferHeader()
{
  header = NULL;
}


unsigned int
SharedMemoryImageBufferHeader::size()
{
  return sizeof(SharedMemoryImageBuffer_header_t);
}


unsigned int
SharedMemoryImageBufferHeader::dataSize()
{
  if (header == NULL) {
    return colorspace_buffer_size(colorspace, width, height);
  } else {
    return colorspace_buffer_size(fuse_ui2cs(header->colorspace), header->width, header->height);
  }
}


bool
SharedMemoryImageBufferHeader::matches(void *memptr)
{
  SharedMemoryImageBuffer_header_t *h = (SharedMemoryImageBuffer_header_t *)memptr;

  if (image_num == 0xFFFFFFFF) {
    return true;

  } else if (h->image_num == image_num) {

    if ( (colorspace == CS_UNKNOWN) ||
	 ((fuse_ui2cs(h->colorspace) == colorspace) &&
	  (h->width == width) &&
	  (h->height == height)
	  )
	 ) {
      return true;
    } else {
      throw InconsistentImageException("Inconsistent image found in memory (meta)");
    }
  } else {
    return false;
  }

}


/** Print some info. */
void
SharedMemoryImageBufferHeader::print_info()
{
  if (header == NULL) {
    cout << "No image set" << endl;
    return;
  }
  cout << "SharedMemory Image Info: " << endl;
  printf("    address:  0x%lx\n", (long unsigned int)header);
  cout << "    image num:  " << header->image_num << endl
       << "    colorspace: " << header->colorspace << endl
       << "    dimensions: " << header->width << "x" << header->height << endl
       << "    ROI:        at (" << header->roi_x << "," << header->roi_y
       << ")  dim " << header->roi_width << "x" << header->roi_height << endl
       << "    circle:     " << (header->flag_circle_found ? "" : "not ")
       << "found at (" << header->circle_x << "," << header->circle_y
       << ")  radius " << header->circle_radius << endl
       << "    img ready:  " << (header->flag_image_ready ? "yes" : "no") << endl; 
}


/** Create if colorspace, width and height have been supplied.
 * @return true if colorspace has been set, width and height are greater than zero.
 */
bool
SharedMemoryImageBufferHeader::create()
{
  return ( (colorspace != CS_UNKNOWN) &&
	   (width > 0) &&
	   (height > 0) );
}


void
SharedMemoryImageBufferHeader::initialize(void *memptr)
{
  header = (SharedMemoryImageBuffer_header_t *)memptr;
  memset(memptr, 0, sizeof(SharedMemoryImageBuffer_header_t));
	 
  header->image_num  = image_num;
  header->colorspace = fuse_cs2ui(colorspace);
  header->width      = width;
  header->height     = height;

}


void
SharedMemoryImageBufferHeader::set(void *memptr)
{
  header = (SharedMemoryImageBuffer_header_t *)memptr;
}


/** Get colorspace.
 * @return colorspace
 */
colorspace_t
SharedMemoryImageBufferHeader::getColorspace()
{
  if (header == NULL) return CS_UNKNOWN;
  return fuse_ui2cs(header->colorspace);
}


/** Get width.
 * @return image width
 */
unsigned int
SharedMemoryImageBufferHeader::getWidth()
{
  if (header == NULL) return 0;
  return header->width;
}


/** Get height.
 * @return image height
 */
unsigned int
SharedMemoryImageBufferHeader::getHeight()
{
  if (header == NULL) return 0;
  return header->height;
}


/** Get image number
 * @return image number
 */
unsigned int
SharedMemoryImageBufferHeader::getImageNumber()
{
  if (header == NULL) return 0;
  return header->image_num;
}


/** Set image number
 * @param image_num image number
 */
void
SharedMemoryImageBufferHeader::setImageNumber(unsigned int image_num)
{
  this->image_num = image_num;
}


/** Get raw header.
 * @return raw header.
 */
SharedMemoryImageBuffer_header_t *
SharedMemoryImageBufferHeader::getRawHeader()
{
  return header;
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
SharedMemoryImageBufferLister::printHeader()
{
  cout << endl << cgreen << "FireVision Shared Memory Segments - Images" << cnormal << endl
       << "========================================================================" << endl
       << cwhite;
  printf ("%-3s %-10s %-10s %-9s %-16s %-5s %-5s %s\n",
          "#", "ShmID", "Semaphore", "Bytes", "Color Space", "Width", "Height", "State");
  cout << cnormal
       << "------------------------------------------------------------------------" << endl;
}


void
SharedMemoryImageBufferLister::printFooter()
{
}


void
SharedMemoryImageBufferLister::printNoSegments()
{
  cout << "No FireVision shared memory segments found" << endl;
}


void
SharedMemoryImageBufferLister::printNoOrphanedSegments()
{
  cout << "No orphaned FireVision shared memory segments found" << endl;
}


void
SharedMemoryImageBufferLister::printInfo(SharedMemoryHeader *header,
					 int shm_id, int semaphore,
					 unsigned int mem_size,
					 void *memptr)
{

  SharedMemoryImageBufferHeader *h = (SharedMemoryImageBufferHeader *)header;

  const char *colorspace = colorspace_to_string(h->getColorspace());

  printf("%-3d %-10d %-10d %-9d %-16s %-5d %-5d %s%s\n",
	 h->getImageNumber(), shm_id, semaphore, mem_size, colorspace,
	 h->getWidth(), h->getHeight(),
	 (SharedMemory::isSwapable(shm_id) ? "S" : ""),
	 (SharedMemory::isDestroyed(shm_id) ? "D" : "")
	 );
}
