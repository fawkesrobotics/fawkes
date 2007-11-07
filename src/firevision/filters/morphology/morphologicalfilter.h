
/***************************************************************************
 *  morphological.h - interface for a morphological filter
 *
 *  Created: Thu Jun 08 09:58:21 2006
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

#ifndef HAVE_IPP
#error "IPP not installed"
#endif

#ifndef __FIREVISION_FILTER_MORPHOLOGY_MORPHOLOGICAL_H_
#define __FIREVISION_FILTER_MORPHOLOGY_MORPHOLOGICAL_H_

#include <filters/filter.h>

class MorphologicalFilter : public Filter {
 public:
  MorphologicalFilter(const char *name, unsigned int max_num_buffers = 1);
  virtual ~MorphologicalFilter();
  virtual void set_structuring_element(unsigned char *se,
				       unsigned int se_width,
				       unsigned int se_height,
				       unsigned int se_anchor_x,
				       unsigned int se_anchor_y );

 protected:
  unsigned char *se;
  unsigned int   se_width;
  unsigned int   se_height;
  unsigned int   se_anchor_x;
  unsigned int   se_anchor_y;
};


#endif
