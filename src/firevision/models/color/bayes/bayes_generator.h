
/**************************************************************************
 *  bayes_generator.h - generator for lookuptables using a bayesian method
 *
 *  Generated: Wed Mar 01 14:00:41 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ***************************************************************************/

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

#ifndef __FIREVISION_MODELS_COLOR_BAYES_BAYES_GENERATOR_
#define __FIREVISION_MODELS_COLOR_BAYES_BAYES_GENERATOR_

#include <models/color/lookuptable_generator.h>

#include <vector>
#include <map>

class ColorModelLookupTable;
class BayesHistosToLut;

class BayesColorLutGenerator : public ColorLutGenerator
{

 public:
  BayesColorLutGenerator( unsigned int lut_width = 1, 
			  unsigned int lut_height = 256,
			  unsigned int lut_depth = 256,
			  hint_t fg_object = H_UNKNOWN);
  ~BayesColorLutGenerator();

  virtual void                     set_fg_object(hint_t object);
  virtual void                     set_buffer(unsigned char *buffer,
					      unsigned int width, unsigned int height);
  virtual ColorModelLookupTable *  get_current();
  virtual void                     consider();
  virtual void                     calc();
  virtual void                     undo();
  virtual void                     reset();
  virtual void                     reset_undo();

  virtual void                     set_selection(std::vector< rectangle_t > region);

  virtual bool                     has_histograms();
  virtual std::map< hint_t, Histogram * > *  get_histograms();

  void set_min_probability(float min_prob);

 private:
  bool is_in_region(unsigned int x, unsigned int y);
  void normalize_histos();

  std::map< hint_t, Histogram * > fg_histos;
  std::map< hint_t, Histogram * > bg_histos;
  std::map< hint_t, Histogram * > histos;
  std::map< hint_t, Histogram * >::iterator histo_it;

  BayesHistosToLut      *bhtl;
  ColorModelLookupTable *cm;

  hint_t fg_object;

  unsigned int lut_width;
  unsigned int lut_height;
  unsigned int lut_depth;

  unsigned int image_width;
  unsigned int image_height;

  unsigned char *buffer;
  std::vector< rectangle_t >  region;
  std::vector< rectangle_t >::iterator  rit;

  bool *selection_mask;
};

#endif
