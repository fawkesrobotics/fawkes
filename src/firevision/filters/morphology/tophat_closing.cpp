
/***************************************************************************
 *  tophat_closing.cpp - implementation of morphological tophat closing
 *
 *  Generated: Sat Jun 10 16:21:30 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
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

#include <filters/morphology/tophat_closing.h>
#include <filters/morphology/segenerator.h>
#include <filters/morphology/closing.h>
#include <filters/difference.h>

#include <cstddef>

/** Image that we subtract from */
const unsigned int FilterTophatClosing::SUBTRACTFROM = 0;
/** Image to filter. */
const unsigned int FilterTophatClosing::FILTERIMAGE  = 1;

#define ERROR(m) {							\
    Exception e("FilterTophatClosing failed");				\
    e.append("Function: %s", __FUNCTION__);				\
    e.append("Message:  %s", m);					\
    throw e;								\
  }

/** @class FilterTophatClosing <filters/morphology/tophat_closing.h>
 * Morphological tophat closing.
 * @author Tim Niemueller
 */

/** Constructor. */
FilterTophatClosing::FilterTophatClosing()
{
  closing  = new FilterClosing();
  diff     = new FilterDifference();

  src[SUBTRACTFROM] = src[FILTERIMAGE] = dst = NULL;
  src_roi[SUBTRACTFROM] = src_roi[FILTERIMAGE] = dst_roi = NULL;
}


/** Destructor. */
FilterTophatClosing::~FilterTophatClosing()
{
  delete closing;
  delete diff;
}


void
FilterTophatClosing::setSrcBuffer(unsigned char *buf, ROI *roi,
				    orientation_t ori, unsigned int buffer_num)
{
  setSrcBuffer(buf, roi, buffer_num);
}


void
FilterTophatClosing::setSrcBuffer(unsigned char *buf, ROI *roi, unsigned int buffer_num)
{
  if ( buffer_num >= 2 ) ERROR("Invalid buffer number");

  src[buffer_num] = buf;
  src_roi[buffer_num] = roi;
}


void
FilterTophatClosing::setDstBuffer(unsigned char *buf, ROI *roi, orientation_t ori)
{
  dst = buf;
  dst_roi = roi;
}


void
FilterTophatClosing::setStructuringElement(unsigned char *se,
					     unsigned int se_width, unsigned int se_height,
					     unsigned int se_anchor_x, unsigned int se_anchor_y)
{
  closing->setStructuringElement( se, se_width, se_height, se_anchor_x, se_anchor_y );
}


void
FilterTophatClosing::setOrientation(orientation_t ori)
{
}


const char *
FilterTophatClosing::getName()
{
  return "FilterTophatClosing";
}

void
FilterTophatClosing::apply()
{
  if ( dst == NULL ) ERROR("dst == NULL");
  if ( src[SUBTRACTFROM] == NULL ) ERROR("src[SUBTRACTFROM] == NULL");
  if ( src[FILTERIMAGE] == NULL ) ERROR("src[FILTERIMAGE] == NULL");
  if ( *(src_roi[SUBTRACTFROM]) != *(src_roi[FILTERIMAGE]) ) ERROR("marker and mask ROI differ");

  closing->setSrcBuffer( src[FILTERIMAGE], src_roi[FILTERIMAGE] );
  closing->setDstBuffer( dst, dst_roi );

  diff->setSrcBuffer( src[SUBTRACTFROM], src_roi[SUBTRACTFROM], 1 );
  diff->setSrcBuffer( dst, dst_roi, 0 );
  diff->setDstBuffer( dst, dst_roi );

  closing->apply();
  diff->apply();
}
