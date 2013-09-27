
/***************************************************************************
 *  histogram.h - Header for histograms
 *
 *  Generated: Tue Jun 14 11:09:27 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_UTILS_HISTOGRAM_H_
#define __FIREVISION_UTILS_HISTOGRAM_H_

#include <fvutils/base/types.h>
#include <iostream>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class HistogramBlock;

class Histogram
{
 public:
  Histogram(unsigned int width, unsigned int height,
      unsigned int depth = 1, unsigned int num_undos = 1);
  Histogram(HistogramBlock* histogram_block);
  ~Histogram();

  void           operator+=(fawkes::upoint_t *p);
  void           operator+=(fawkes::upoint_t  p);

  unsigned int * get_histogram();
  HistogramBlock* get_histogram_block();
  void           get_dimensions(unsigned int& width, unsigned int& height, unsigned int& depth);
  unsigned int   get_value(unsigned int x, unsigned int y);
  unsigned int   get_value(unsigned int x, unsigned int y, unsigned int z);
  void           set_value(unsigned int x, unsigned int y, unsigned int value);
  void           set_value(unsigned int x, unsigned int y, unsigned int z, unsigned int value);
  void           inc_value(unsigned int x, unsigned int y, unsigned int z = 0);
  void           add(unsigned int x, unsigned int y, unsigned int z, unsigned int value);
  void           sub(unsigned int x, unsigned int y, unsigned int z, unsigned int value);
  void           reset();
  unsigned int   get_median();
  unsigned int   get_average();

  unsigned int   get_sum() const;

  void           reset_undo();
  void           undo();
  unsigned int   switch_undo( unsigned int undo_id );
  unsigned int   get_num_undos();

  void           print_to_stream(std::ostream &s);
  void           save(const char * filename, bool formatted_output = false);
  bool           load(const char * filename);

 private:
  unsigned int  width;
  unsigned int  height;
  unsigned int  depth;
  unsigned int  dimension;
  unsigned int  histogram_size;
  unsigned int* histogram;
  HistogramBlock *histogram_block;
  unsigned int  number_of_values;

  unsigned int **undo_overlay;
  unsigned int  *undo_num_vals;
  unsigned int   undo_num;
  unsigned int   undo_current;
};

} // end namespace firevision

#endif
