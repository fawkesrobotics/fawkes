
/***************************************************************************
 *  rectify.h - Header of rectification filter
 *
 *  Created: Wed Nov 07 11:07:59 2007
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

#ifndef __FIREVISION_FILTER_RECTIFY_H_
#define __FIREVISION_FILTER_RECTIFY_H_

#include <filters/filter.h>

class RectificationInfoBlock;

class FilterRectify : public Filter
{
 public:
  FilterRectify(RectificationInfoBlock *rib, bool mark_zeros = true);

  virtual void apply();

 private:
  RectificationInfoBlock *__rib;
  bool __mark_zeros;
};

#endif

