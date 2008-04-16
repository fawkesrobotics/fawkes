
/***************************************************************************
 *  roidraw.h - Header of ROI draw filter
 *
 *  Created: Thu Jul 14 15:59:22 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_FILTER_ROIDRAW_H_
#define __FIREVISION_FILTER_ROIDRAW_H_

#include <filters/filter.h>
#include <list>

class ROI;

class FilterROIDraw : public Filter
{
 public:
  FilterROIDraw(std::list<ROI> *rois = NULL);

  virtual void apply();

  void set_rois(std::list<ROI> *rois);

 private:
  void draw_roi(ROI *roi);

  std::list<ROI> *__rois;
};

#endif
