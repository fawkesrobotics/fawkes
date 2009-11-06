
/**************************************************************************
 *  bayes_generator.h - generator for colormap using a bayesian method
 *
 *  Created: Wed Mar 01 14:00:41 2006
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_MODELS_COLOR_BAYES_BAYES_GENERATOR_
#define __FIREVISION_MODELS_COLOR_BAYES_BAYES_GENERATOR_

#include <fvutils/colormap/generator.h>

#include <vector>
#include <map>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class YuvColormap;
class BayesHistosToLut;

class BayesColormapGenerator : public ColormapGenerator
{

 public:
  BayesColormapGenerator( unsigned int lut_depth = 1,
			  hint_t fg_object = H_UNKNOWN,
			  unsigned int lut_width = 256, unsigned int lut_height = 256);
  ~BayesColormapGenerator();

  virtual void                     set_fg_object(hint_t object);
  virtual void                     set_buffer(unsigned char *buffer,
					      unsigned int width, unsigned int height);
  virtual YuvColormap *            get_current();
  virtual void                     consider();
  virtual void                     calc();
  virtual void                     undo();
  virtual void                     reset();
  virtual void                     reset_undo();

  virtual void                     set_selection(std::vector< fawkes::rectangle_t > region);

  virtual bool                     has_histograms();
  virtual std::map< hint_t, Histogram * > *  get_histograms();

  virtual void                     load_histograms(const char *filename);
  virtual void                     save_histograms(const char *filename);

  void set_min_probability(float min_prob);

 private:
  bool is_in_region(unsigned int x, unsigned int y);
  void normalize_histos();

  typedef std::map< hint_t, Histogram * > HistogramMap;
  HistogramMap fg_histos;
  HistogramMap bg_histos;
  HistogramMap histos;
  HistogramMap::iterator histo_it;

  BayesHistosToLut      *bhtl;
  YuvColormap           *cm;

  hint_t fg_object;

  unsigned int lut_width;
  unsigned int lut_height;
  unsigned int lut_depth;

  unsigned int image_width;
  unsigned int image_height;

  unsigned int norm_size;

  unsigned char *buffer;
  std::vector< fawkes::rectangle_t >  region;
  std::vector< fawkes::rectangle_t >::iterator  rit;

  bool *selection_mask;
};

} // end namespace firevision

#endif
