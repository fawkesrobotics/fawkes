
/***************************************************************************
 *  nothing.h - Header of a filter that does nothing. Wonderful, isn't it.
 *              Used in FilterFactory to avoid NULL checks
 *
 *  Created: Fri Jun 17 13:28:21 2005
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

#ifndef __FIREVISION_FILTER_NOTHING_H_
#define __FIREVISION_FILTER_NOTHING_H_

#include <filters/filter.h>

/** @class FilterNothing <filters/nothing.h>
 * A filter that does... nothing.
 * @author Tim Niemueller
 */

class FilterNothing : public Filter
{
 public:
  /** Constructor. */
  FilterNothing() : Filter("FilterNothing") {}

  virtual void apply() {}
};

#endif
