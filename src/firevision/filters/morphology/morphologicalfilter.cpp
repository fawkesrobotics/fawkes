
/***************************************************************************
 *  morphological.cpp - interface for a morphological filter
 *
 *  Generated: Tue Mar 27 23:27:46 2007
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

#include <filters/morphology/morphologicalfilter.h>

/** @class MorphologicalFilter <filters/morphology/morphologicalfilter.h>
 * Morphological filter interface.
 * This interface defines specific API details for morphological filters.
 *
 * @author Tim Niemueller
 *
 * @fn MorphologicalFilter::setStructuringElement(unsigned char *se, unsigned int se_width, unsigned int se_height, unsigned int se_anchor_x, unsigned int se_anchor_y )
 * Set the structuring element for successive filter runs.
 * @param se structuring element buffer. This is just a line-wise concatenated array
 * of values. A value of zero means ignore, any other value means to consider this
 * value.
 * @param se_width width of structuring element
 * @param se_height height of structuring element
 * @param se_anchor_x x coordinate of anchor in structuring element
 * @param se_anchor_y y coordinate of anchor in structuring element
 */

/** Virtual empty destructor. */
MorphologicalFilter::~MorphologicalFilter()
{
}
