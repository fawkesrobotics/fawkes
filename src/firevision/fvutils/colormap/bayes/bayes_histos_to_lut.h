
/**************************************************************************
 *  bayes_histos_to_lut.h - This header defines a class
 *                          that takes color histograms of objects as input,
 *                          and, together with probabilities of objects,
 *                          generates all the values for a lookup-table
 *                          that maps from colors to objects
 *
 *  Created: Mon Jun 27 14:16:52 2005
 *  Copyright  2005       Martin Heracles
 *             2005-2008  Tim Niemueller [www.niemueller.de]
 *             2007-2008  Daniel Beck
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

#ifndef __FIREVISION_COLORMODEL_BAYES_HISTOS_TO_LUT_H_
#define __FIREVISION_COLORMODEL_BAYES_HISTOS_TO_LUT_H_

#include <fvutils/base/roi.h>

#include <map>
#include <string>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Histogram;
class YuvColormap;

class BayesHistosToLut
{
 public:
  BayesHistosToLut(std::map< hint_t, Histogram * > &histos,
		   unsigned int d = 1,
		   hint_t fg_object = H_UNKNOWN,
		   unsigned int w = 256,
		   unsigned int h = 256);
  ~BayesHistosToLut();

  std::string getName();

  float getObjectProb(hint_t object);

  float getAPrioriProb( unsigned int u,
			unsigned int v,
			hint_t object );
  float getAPrioriProb( unsigned int y,
			unsigned int u,
			unsigned int v,
			hint_t object );

  float getAPosterioriProb( hint_t object,
			    unsigned int u,
			    unsigned int v );
  float getAPosterioriProb( hint_t object,
			    unsigned int y,
			    unsigned int u,
			    unsigned int v );

  hint_t getMostLikelyObject( unsigned int u,
			      unsigned int v );
  hint_t getMostLikelyObject( unsigned int y,
			      unsigned int u,
			      unsigned int v );

  void setMinProbability( float min_prob );
  void setMinProbForColor( float min_prob, hint_t hint );

  YuvColormap * get_colormap();

  /* method "calculateLutValues" calculates lut values
     following the bayesian approach */
  void calculateLutValues( bool penalty = false );
  /* method "calculateLutAllColors" calculates lut values
     _without_ following the bayesian approach, but it can handle all colors
     (not only "ball" and "background") */
  void calculateLutAllColors();
  void saveLut(char *file);
  void save(std::string filename);

 private:
  std::map<hint_t, Histogram*>  &histograms;
  std::map<hint_t, unsigned int> numberOfOccurrences;
  std::map<hint_t, float>        object_probabilities;

  YuvColormap *lut;
  unsigned int width;
  unsigned int height;
  unsigned int depth;

  hint_t fg_object;

  float   min_probability;

  // color thresholds:
  float min_prob_ball;
  float min_prob_green;
  float min_prob_yellow;
  float min_prob_blue;
  float min_prob_white;
  float min_prob_black;
};

} // end namespace firevision

#endif
