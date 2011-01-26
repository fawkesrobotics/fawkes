
/***************************************************************************
 *  filter.cpp - Abstract class defining a filter
 *
 *  Created: Mon May 19 15:47:44 2007
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

#include <fvfilters/filter.h>

#include <core/exceptions/software.h>
#include <cstdlib>
#include <cstring>

using namespace fawkes;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Filter <fvfilters/filter.h>
 * Filter interface.
 * This class defines the general interface that filters are used with.
 *
 * @author Tim Niemueller
 *
 * @fn void Filter::apply() = 0
 * Apply the filter.
 * Apply the filter to the given source and destination
 * buffers with given width and height and orientation
 * (ori may be ignored for some filters).
 */

/** Constructor.
 * @param name name of the filter
 * @param max_num_buffers The maximum number of source buffers that can be set.
 */
Filter::Filter(const char *name, unsigned int max_num_buffers)
{
  if ( max_num_buffers == 0 ) {
    throw OutOfBoundsException("Need to set at least one buffer", 0, 1, 0xFFFFFFFF);
  }

  _name = strdup(name);
  _max_num_buffers = max_num_buffers;

  src = (unsigned char **)malloc(_max_num_buffers * sizeof(unsigned char *));
  memset(src, 0, _max_num_buffers * sizeof(unsigned char *));

  src_roi = (ROI **)malloc(_max_num_buffers * sizeof(ROI *));
  memset(src_roi, 0, _max_num_buffers * sizeof(ROI *));

  ori = (orientation_t *)malloc(_max_num_buffers * sizeof(orientation_t));
  memset(ori, 0, _max_num_buffers * sizeof(orientation_t));
}


/** Destructor. */
Filter::~Filter()
{
  free(_name);
  free(src);
  free(src_roi);
  free(ori);
}

/** Set source buffer with orientation.
 * @param buf Buffer to use as source image
 * @param roi Region Of Interest to work on
 * @param ori Orientation to apply the filter in, maybe ignored
 *            in some filters
 * @param buffer_num source buffer to set for filter that need
 *                   multiple src buffers
 * @exception OutOfBoundsException Thrown if buffer_num is illegal
 */
void
Filter::set_src_buffer(unsigned char *buf,
		       ROI *roi,
		       orientation_t ori,
		       unsigned int buffer_num)
{
  if ( buffer_num >= _max_num_buffers ) {
    throw OutOfBoundsException("Invalid buffer number", buffer_num, 0, _max_num_buffers);
  }

  src[buffer_num]       = buf;
  src_roi[buffer_num]   = roi;
  this->ori[buffer_num] = ori;
}


/** Set source buffer.
 * @param buf Buffer to use as source image
 * @param roi Region Of Interest to work on
 * @param buffer_num source buffer to set for filter that need multiple src buffers
 * @exception OutOfBoundsException Thrown if buffer_num is illegal
 */
void
Filter::set_src_buffer(unsigned char *buf,
		       ROI *roi,
		       unsigned int buffer_num)
{
  if ( buffer_num >= _max_num_buffers ) {
    throw OutOfBoundsException("Invalid buffer number", buffer_num, 0, _max_num_buffers);
  }

  src[buffer_num]     = buf;
  src_roi[buffer_num] = roi;
  ori[buffer_num]     = ORI_HORIZONTAL;
}


/** Set the destination buffer.
 * @param buf Buffer to use as destination image
 * @param roi Region Of Interest where the result is put in the dst image
 */
void
Filter::set_dst_buffer(unsigned char *buf, ROI *roi)
{
  dst     = buf;
  dst_roi = roi;
}


/** Set the orientation to apply the filter in.
 * Maybe ignored by some filters.
 * @param ori Orientation
 * @param buffer_num buffer this orientation applies to
 */
void
Filter::set_orientation(orientation_t ori, unsigned int buffer_num)
{
  if ( buffer_num >= _max_num_buffers ) {
    throw OutOfBoundsException("Invalid buffer number", buffer_num, 0, _max_num_buffers);
  }

  this->ori[buffer_num] = ORI_HORIZONTAL;
}


/** Get filter name
 * @return filter name
 */
const char *
Filter::name()
{
  return _name;
}


/** This shrinks the regions as needed for a N x N matrix.
 * @param r ROI to shrink
 * @param n size of the matrix
 */
void
Filter::shrink_region(ROI *r, unsigned int n)
{
  if (r->start.x < (n/2)) {
    r->start.x = n/2;
  }
  if (r->start.y < (n/2)) {
    r->start.y = n/2;
  }
  if ( (r->start.x + r->width) >= (r->image_width - (n/2)) ) {
    r->width -= (r->start.x + r->width) - (r->image_width - (n/2));
  }
  if ( (r->start.y + r->height) >= (r->image_height - (n/2)) ) {
    r->height -= (r->start.y + r->height) - (r->image_height - (n/2));
  }
}

} // end namespace firevision
