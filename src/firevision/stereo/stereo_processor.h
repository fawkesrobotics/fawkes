
/***************************************************************************
 *  stereo_processor.h - Stereo processor interface
 *
 *  Created: Fri May 18 15:58:30 2007
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_FVUTILS_STEREO_STEREO_PROCESSOR_H_
#define __FIREVISION_FVUTILS_STEREO_STEREO_PROCESSOR_H_

#include <sys/types.h>

class ROI;

class StereoProcessor
{
 public:
  virtual ~StereoProcessor();

  virtual bool get_xyz(unsigned int px, unsigned int py,
		       float *x, float *y, float *z)                     = 0;

  virtual bool get_world_xyz(unsigned int px, unsigned int py,
			     float *x, float *y, float *z)               = 0;

  virtual void             preprocess_stereo()                           = 0;
  virtual void             calculate_disparity(ROI *roi = 0)             = 0;
  virtual void             calculate_yuv(bool both = false)              = 0;

  virtual unsigned char *  disparity_buffer()                            = 0;
  virtual size_t           disparity_buffer_size() const                 = 0;
  virtual unsigned char *  yuv_buffer()                                  = 0;
  virtual unsigned char *  auxiliary_yuv_buffer()                        = 0;
};

#endif
