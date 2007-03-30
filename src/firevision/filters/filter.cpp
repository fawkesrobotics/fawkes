
/***************************************************************************
 *  filter.cpp - Abstract class defining a filter
 *
 *  Generated: Mon May 19 15:47:44 2007
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

#include <filters/filter.h>

/** @class Filter <filters/filter.h>
 * Filter interface.
 * This class defines the general interface that filters are used with.
 *
 * @author Tim Niemueller
 *
 * @fn void Filter::setSrcBuffer(unsigned char *buf, ROI *roi, orientation_t ori = ORI_HORIZONTAL, unsigned int buffer_num = 0) = 0
 * Set the src buffer
 * @param buf Buffer to use as source image
 * @param roi Region Of Interest to work on
 * @param ori Orientation to apply the filter in, maybe ignored
 *            in some filters
 * @param buffer_num source buffer to set for filter that need
 *                   multiple src buffers
 *
 * @fn void Filter::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num) = 0
 * Set the src buffer
 * @param buf Buffer to use as source image
 * @param roi Region Of Interest to work on
 * @param buffer_num source buffer to set for filter that need
 *                   multiple src buffers
 *
 * @fn void Filter::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori = ORI_HORIZONTAL) = 0
 * Set the destination buffer.
 * @param buf Buffer to use as destination image
 * @param roi Region Of Interest where the result is put in the dst image
 * @param ori Orientation to apply the filter in, maybe ignored
 *            in some filters
 *
 * @fn void Filter::setOrientation(orientation_t ori) = 0
 * Set the orientation to apply the filter in, maybe ignored by
 * some filters.
 * @param ori Orientation
 *
 * @fn void Filter::apply() = 0
 * Apply the filter.
 * Apply the filter to the given source and destination
 * buffers with given width and height and orientation
 * (Ori may be ignored for some filters).
 *
 * @fn const char * Filter::getName() = 0
 * Get name of filter.
 * @return name of filter
 */


/** Empty virtual destructor. */
Filter::~Filter()
{
}

/** This shinks the regions as needed for a N x N matrix.
 * @param r ROI to shrink
 * @param n size of the matrix
 */
void
Filter::shrinkRegion(ROI *r, unsigned int n)
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

