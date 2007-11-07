
/***************************************************************************
 *  filter.h - Abstract class defining a filter
 *
 *  Created: Tue May 03 19:50:02 2005
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

#ifndef __FIREVISION_FILTER_H_
#define __FIREVISION_FILTER_H_

#include <fvutils/base/types.h>
#include <fvutils/base/roi.h>

class Filter
{

 public:
  Filter(const char *name, unsigned int max_num_buffers = 1);
  virtual ~Filter();

  virtual void set_src_buffer(unsigned char *buf, ROI *roi,
			      orientation_t ori = ORI_HORIZONTAL,
			      unsigned int buffer_num = 0);

  virtual void set_src_buffer(unsigned char *buf, ROI *roi,
			      unsigned int buffer_num);

  virtual void set_dst_buffer(unsigned char *buf, ROI *roi);


  virtual void set_orientation(orientation_t ori, unsigned int buffer_num);
  virtual const char * name();

  virtual void apply() = 0 ;

  void shrink_region(ROI *r, unsigned int n);

 protected:
  unsigned int    _max_num_buffers;
  char           *_name;

  unsigned char  **src;
  unsigned char  *dst;

  ROI            **src_roi;
  ROI            *dst_roi;

  orientation_t  *ori;
};

#endif
