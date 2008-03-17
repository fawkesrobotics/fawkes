
/**************************************************************************
 *  bayes_generator.cpp - generator for lookuptables using a bayesian method
 *
 *  Generated: Wed Mar 01 14:14:41 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *             2007       Daniel Beck
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

#include <models/color/bayes/bayes_generator.h>

#include <fvutils/color/yuv.h>
#include <fvutils/statistical/histogram.h>
#include <models/color/lookuptable.h>
#include <models/color/bayes/bayes_histos_to_lut.h>

using namespace std;

/** @class BayesColorLutGenerator <models/color/bayes/bayes_generator.h>
 * Color LUT Generator using Bayes method.
 * @author Tim Niemueller
 * @author Daniel Beck
 */

/** Constructor. 
 * @param fg_object the type of the foreground object
 */
BayesColorLutGenerator::BayesColorLutGenerator(hint_t fg_object)
{
  this->fg_object = fg_object;

  lut_width  = 256;
  lut_height = 256;
  lut_depth  =   1;

  histos.clear();
  image_width = image_height = 0;

  Histogram *h_fg = new Histogram(lut_width, lut_height);
  // for bg histo we want extra undo for penalty, thus 2
  Histogram *h_bg = new Histogram(lut_width, lut_height, lut_depth, 2);
  
  /* not yet
  Histogram *h_black = new Histogram(256, 256);
  Histogram *h_green = new Histogram(256, 256);
  Histogram *h_yellow = new Histogram(256, 256);
  Histogram *h_blue = new Histogram(256, 256);
  Histogram *h_white = new Histogram(256, 256);
  */

  // The order you push them into histos is important! It
  // _must_ follow hint_t, you cannot leave histos out!
  histos[fg_object] = h_fg;
  histos[H_BACKGROUND] = h_bg;

  /*
  vector< Histogram * > histos_v;
  histos_v.clear();
  histos_v.push_back( h_ball );
  histos_v.push_back( h_bg );
  */

  bhtl = new BayesHistosToLut(histos, lut_width, lut_height, fg_object);
  cm = bhtl->getColorModel();
}


/** Set buffer.
 * @param buffer image buffer
 * @param width image width
 * @param height image height
 */
void
BayesColorLutGenerator::setBuffer(unsigned char *buffer,
				  unsigned int width, unsigned int height)
{
  this->buffer = buffer;
  image_width = width;
  image_height = height;
}


/** Get current color model.
 * @return current color model
 */
ColorModelLookupTable *
BayesColorLutGenerator::getCurrent()
{
  return cm;
}


/** Check if pixel is in region.
 * @param x image x coordinate
 * @param y image y coordinate
 * @return true if pixel is in region, false otherwise
 */
bool
BayesColorLutGenerator::isInRegion(unsigned int x, unsigned int y) {

  vector<rectangle_t>::iterator it;

  // compare (x, y) to the slices in region
  for (it = region.begin(); it != region.end(); it++) {
    if (((*it).start.x <= x) &&
	(x <= (*it).start.x + (*it).extent.w) &&
	((*it).start.y <= y) &&
	(y <= (*it).start.y + (*it).extent.h)) {
      return true;
    }
  }
  return false;
}


/** Set selection.
 * @param region selected region.
 */
void
BayesColorLutGenerator::setSelection(vector< rectangle_t > region)
{
  this->region = region;
}


/** Set min probability.
 * @param min_prob min probability.
 * @see BayesHistosToLut::setMinProbability()
 */
void
BayesColorLutGenerator::setMinProbability(float min_prob)
{
  bhtl->setMinProbability( min_prob );
}


/** Consider current image. */
void
BayesColorLutGenerator::consider()
{

  if (region.size() == 0) {
    cout << "Region empty, cannot consider" << endl;
    return;
  }

  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    (*histo_it).second->reset_undo();
  }

  point_t p;

  for (unsigned int w = 0; w < image_width; ++w) {
    for (unsigned int h = 0; h < image_height; ++h) {
      p.x = YUV422_PLANAR_U_AT(buffer, image_width, image_height, w, h);
      p.y = YUV422_PLANAR_V_AT(buffer, image_width, image_height, w, h);
      if (isInRegion(w, h)) {
	*(histos[fg_object]) += p;
      }	else {
	*(histos[H_BACKGROUND]) += p;
      }
    }
  }
}


/** Calculate. */
void
BayesColorLutGenerator::calc()
{
  bhtl->calculateLutValues( false /* no penalty*/ );
}


/** Undo last inclusion. */
void
BayesColorLutGenerator::undo()
{
  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    (*histo_it).second->undo();
  }
}


/** Reset color model. */
void
BayesColorLutGenerator::reset()
{
  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    (*histo_it).second->reset();
  }
  cm->reset();
}


/** Reset undo. */
void
BayesColorLutGenerator::resetUndo()
{
  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    (*histo_it).second->reset_undo();
  }
}


/** Check if this color model uses histograms.
 * @return true
 */
bool
BayesColorLutGenerator::hasHistograms()
{
  return true;
}


/** Get histograms.
 * @return histograms
 */
std::map< hint_t, Histogram * > *
BayesColorLutGenerator::getHistograms()
{
  return &histos;
}
