
/***************************************************************************
 *  segment_scanline.h - Header of color segmentation filter
 *
 *  Created: Thu Jul 14 15:01:59 2005
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

#ifndef __FIREVISION_FILTER_SCANLINESEGMENTATION_H_
#define __FIREVISION_FILTER_SCANLINESEGMENTATION_H_

#include <fvfilters/filter.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ColorModel;
class ScanlineModel;

class FilterScanlineSegmentation : public Filter
{
 public:
  FilterScanlineSegmentation(ColorModel *cm, ScanlineModel *slm);

  virtual void apply();

 private:
  ColorModel    *cm;
  ScanlineModel *slm;
};

} // end namespace firevision

#endif
