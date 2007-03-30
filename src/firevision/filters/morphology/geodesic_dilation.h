
/***************************************************************************
 *  geodesic_dilation.h - header for morphological geodesic dilation
 *                        reconstruction
 *
 *  Generated: Wed Jun 21 16:25:22 2006
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

#ifndef __FIREVISION_FILTERS_MORPHOLOGY_GEODESIC_DILATION_H_
#define __FIREVISION_FILTERS_MORPHOLOGY_GEODESIC_DILATION_H_

#include <filters/morphology/morphologicalfilter.h>

class FilterDilation;
class FilterMin;
class ImageDiff;
class ROI;

class FilterGeodesicDilation : public MorphologicalFilter
{
 public:
  FilterGeodesicDilation(unsigned int se_size = 3);
  virtual ~FilterGeodesicDilation();

  virtual void setSrcBuffer(unsigned char *buf,
			    ROI *roi,
			    orientation_t ori = ORI_HORIZONTAL,
			    unsigned int buffer_num = 0);

  virtual void setSrcBuffer(unsigned char *buf,
			    ROI *roi,
			    unsigned int buffer_num);

  virtual void setDstBuffer(unsigned char *buf,
			    ROI *roi,
			    orientation_t ori = ORI_HORIZONTAL);

  virtual void setOrientation(orientation_t ori);

  virtual void setStructuringElement(unsigned char *se,
				     unsigned int se_width, unsigned int se_height,
				     unsigned int se_anchor_x, unsigned int se_anchor_y);

  virtual void apply();

  virtual const char *  getName();

  virtual unsigned int getNumIterations();

  static const unsigned int MARKER;
  static const unsigned int MASK;

 private:
  unsigned char *src[2];
  unsigned char *dst;

  unsigned char *isotropic_se;
  unsigned int   se_size;

  ROI           *src_roi[2];
  ROI           *dst_roi;

  FilterDilation *dilate;
  FilterMin      *min;

  ImageDiff      *diff;

  unsigned int    iterations;

};

#endif
