
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

#include <fvutils/base/types.h>

#include <models/color/lookuptable_generator.h>

#include <vector>
#include <map>

class ColorModelLookupTable;
class BayesHistosToLut;
class Histogram2D;

class BayesColorLutGenerator : public ColorLutGenerator
{

 public:
  BayesColorLutGenerator();

  virtual void                     setBuffer(unsigned char *buffer,
					     unsigned int width, unsigned int height);
  virtual ColorModelLookupTable *  getCurrent();
  virtual void                     consider();
  virtual void                     calc();
  virtual void                     undo();
  virtual void                     reset();
  virtual void                     resetUndo();

  virtual void                     setSelection(std::vector< rectangle_t > region);

  virtual bool                     hasHistograms();
  virtual std::map< std::string, Histogram2D * > *  getHistograms();

  void setMinProbability(float min_prob);

 private:
  bool isInRegion(unsigned int x, unsigned int y);


  std::map< std::string, Histogram2D * > histos;
  std::map< std::string, Histogram2D * >::iterator histo_it;

  BayesHistosToLut      *bhtl;
  ColorModelLookupTable *cm;

  unsigned int lut_width;
  unsigned int lut_height;

  unsigned int image_width;
  unsigned int image_height;

  unsigned char *buffer;
  std::vector< rectangle_t >  region;
  std::vector< rectangle_t >::iterator  rit;

};

#endif
