
/***************************************************************************
 *  roidraw.h - Header of ROI draw filter
 *
 *  Created: Thu Jul 14 15:59:22 2005
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvfilters/filter.h>
#include <list>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ROI;
class Drawer;

class FilterROIDraw : public Filter
{
 public: // Typedefs
  /** Defines the possible border styles to display a ROI */
  typedef enum {
    INVERTED,    /**< Displays border with inverted Y-value */
    DASHED_HINT  /**< Displays border dashed black and color of hint*/
  } border_style_t;

 public:
  FilterROIDraw(const std::list<ROI> *rois = 0, border_style_t style = INVERTED);
  virtual ~FilterROIDraw();

  virtual void apply();

  void set_rois(const std::list<ROI> *rois);
  void set_style(border_style_t style);

 private:
  void draw_roi(const ROI *roi);

  const std::list<ROI> *__rois;
  Drawer *__drawer;
  border_style_t __border_style;
};

} // end namespace firevision

#endif
