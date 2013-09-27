/***************************************************************************
 *  gradient.h - Class defining a gradient (color) classifier
 *
 *  Created: Tue Jun 10 11:48:00 2008
 *  Copyright  2008 Christof Rath <christof.rath@gmail.com>
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

#ifndef __FIREVISION_CLASSIFIERS_GRADIENT_H_
#define __FIREVISION_CLASSIFIERS_GRADIENT_H_

#include <fvclassifiers/classifier.h>
#include <fvclassifiers/qualifiers.h>

#include <fvmodels/scanlines/grid.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class GradientClassifier: public Classifier
{
 public:
  GradientClassifier(std::list<ScanlineGrid* >* scanlines, Qualifier* q,
                     unsigned int threshold, unsigned int max_size = 0,
                     bool use_rising_flank = true,
                     bool use_falling_flank = true);
  virtual ~GradientClassifier();

  virtual std::list< ROI > * classify();
  virtual void set_src_buffer(unsigned char *yuv422_planar,
                              unsigned int width, unsigned int height);

  virtual void set_threshold(unsigned int threshold, unsigned int max_size = 0);
  virtual void set_edges(bool use_rising_edge, bool use_falling_edge);

 private:
  int             _last_val;
  fawkes::upoint_t _last_pos;

  unsigned int _threshold;
  unsigned int _max_size;

  std::list<ScanlineGrid* >* _scanlines;
  Qualifier* _q;

  bool _use_falling_edge;
  bool _use_rising_edge;
};

} // end namespace firevision

#endif // __FIREVISION_CLASSIFIERS_GRADIENT_H_
