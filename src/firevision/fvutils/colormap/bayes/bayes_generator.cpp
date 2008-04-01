
/**************************************************************************
 *  bayes_generator.cpp - generator for colormaps using a bayesian method
 *
 *  Created: Wed Mar 01 14:14:41 2006
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *             2007-2008  Daniel Beck
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

#include <fvutils/colormap/bayes/bayes_generator.h>

#include <fvutils/color/yuv.h>
#include <fvutils/statistical/histogram.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/colormap/bayes/bayes_histos_to_lut.h>
#include <core/exception.h>

#include <cmath>

using namespace std;

/** @class BayesColormapGenerator <fvutils/colormap/bayes/bayes_generator.h>
 * Colormap Generator using Bayes method.
 * @author Tim Niemueller
 * @author Daniel Beck
 */

/** Constructor. 
 * @param lut_depth the depth of the lookup table
 * @param fg_object the type of a foreground object
 */
BayesColormapGenerator::BayesColormapGenerator(unsigned int lut_depth,	hint_t fg_object)
{
  this->lut_width  = 256;
  this->lut_height = 256;
  this->lut_depth  = lut_depth;

  set_fg_object(fg_object);

  histos.clear();
  fg_histos.clear();
  bg_histos.clear();

  image_width = image_height = 0;
  selection_mask = 0;

  bhtl = new BayesHistosToLut(histos, lut_depth, fg_object);
  cm = bhtl->get_colormap();
}


/** Destructor. */
BayesColormapGenerator::~BayesColormapGenerator()
{
  for (histo_it = fg_histos.begin(); histo_it != fg_histos.end(); ++histo_it) {
    delete histo_it->second;
  }

  for (histo_it = bg_histos.begin(); histo_it != bg_histos.end(); ++histo_it) {
    delete histo_it->second;
  }

  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    delete histo_it->second;
  }

  delete[] selection_mask;
}


/** Set foreground object.
 * @param object the new foreground object
 */
void
BayesColormapGenerator::set_fg_object(hint_t object)
{
  if (H_UNKNOWN == object)
    { return; }

  if ( fg_histos.find(object) == fg_histos.end() ) {
    fg_histos[object] = new Histogram(lut_width, lut_height, lut_depth);
    bg_histos[object] = new Histogram(lut_width, lut_height, lut_depth, 2);
    histos[object] = new Histogram(lut_width, lut_height, lut_depth);
  }

  fg_object = object;
}


/** Set buffer.
 * @param buffer image buffer
 * @param width image width
 * @param height image height
 */
void
BayesColormapGenerator::set_buffer(unsigned char *buffer,
				   unsigned int width, unsigned int height)
{
  this->buffer = buffer;
  image_width = width;
  image_height = height;

  selection_mask = new bool[image_width * image_height];
  
  for (unsigned int i = 0; i < image_width * image_height; ++i) {
    selection_mask[i] = false;
  }
}


/** Get current color model.
 * @return current color model
 */
YuvColormap *
BayesColormapGenerator::get_current()
{
  return cm;
}


/** Check if pixel is in region.
 * @param x image x coordinate
 * @param y image y coordinate
 * @return true if pixel is in region, false otherwise
 */
bool
BayesColormapGenerator::is_in_region(unsigned int x, unsigned int y) 
{
  return selection_mask[image_width * y + x];
}


/** Set selection.
 * @param region selected region.
 */
void
BayesColormapGenerator::set_selection(vector< rectangle_t > region)
{
  this->region = region;

  for (unsigned int i = 0; i < image_width * image_height; ++i) {
    selection_mask[i] = false;
  }

  vector<rectangle_t>::iterator it;

  // store selection in selection mask
  for (it = region.begin(); it != region.end(); it++) {
    for (unsigned int w = 0; w < (*it).extent.w; ++w) {
      for (unsigned int h = 0; h < (*it).extent.h; ++h) {
	unsigned int x = (*it).start.x + w;
	unsigned int y = (*it).start.y + h;

	selection_mask[image_width * y + x] = true;
      }
    }
  }
}


/** Set min probability.
 * @param min_prob min probability.
 * @see BayesHistosToLut::setMinProbability()
 */
void
BayesColormapGenerator::set_min_probability(float min_prob)
{
  bhtl->setMinProbability( min_prob );
}


/** Consider current image. */
void
BayesColormapGenerator::consider()
{

  if (region.size() == 0) {
    cout << "Region empty, cannot consider" << endl;
    return;
  }

  for (histo_it = fg_histos.begin(); histo_it != fg_histos.end(); ++histo_it) {
    (*histo_it).second->reset_undo();
  }

  for (histo_it = bg_histos.begin(); histo_it != bg_histos.end(); ++histo_it) {
    (*histo_it).second->reset_undo();
  }

  unsigned int y;
  unsigned int u;
  unsigned int v;

  for (unsigned int w = 0; w < image_width; ++w) {
    for (unsigned int h = 0; h < image_height; ++h) {

      y = YUV422_PLANAR_Y_AT(buffer, image_width, w, h);
      u = YUV422_PLANAR_U_AT(buffer, image_width, image_height, w, h);
      v = YUV422_PLANAR_V_AT(buffer, image_width, image_height, w, h);

      unsigned int y_index;
      unsigned int u_index;
      unsigned int v_index;
      
      y_index = (unsigned int)( y / 256.0f * float(lut_width) );
      u_index = (unsigned int)( u / 256.0f * float(lut_height) );
      v_index = (unsigned int)( v / 256.0f * float(lut_depth) );
      
      if ( is_in_region(w, h) ) {
	fg_histos[fg_object]->inc_value(y_index, u_index, v_index);
      }	else {
	bg_histos[fg_object]->inc_value(y_index, u_index, v_index);
      }
    }
    cout << "." << flush;
  }
  cout << endl;
}


/** Calculate. */
void
BayesColormapGenerator::calc()
{
  normalize_histos();
  bhtl->calculateLutValues( false /* no penalty*/ );
}


/** Undo last inclusion. */
void
BayesColormapGenerator::undo()
{
  for (histo_it = fg_histos.begin(); histo_it != fg_histos.end(); ++histo_it) {
    (*histo_it).second->undo();
  }

  for (histo_it = bg_histos.begin(); histo_it != bg_histos.end(); ++histo_it) {
    (*histo_it).second->undo();
  }

  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    (*histo_it).second->undo();
  }
}


/** Reset color model. */
void
BayesColormapGenerator::reset()
{
  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    (*histo_it).second->reset();
  }

  for (histo_it = fg_histos.begin(); histo_it != fg_histos.end(); ++histo_it) {
    (*histo_it).second->reset();
  }

  for (histo_it = bg_histos.begin(); histo_it != bg_histos.end(); ++histo_it) {
    (*histo_it).second->reset();
  }

  cm->reset();
  
  for (unsigned int i = 0; i < image_width * image_height; ++i) {
    selection_mask[i] = false;
  }
}


/** Reset undo. */
void
BayesColormapGenerator::reset_undo()
{
  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    (*histo_it).second->reset_undo();
  }

  for (histo_it = fg_histos.begin(); histo_it != fg_histos.end(); ++histo_it) {
    (*histo_it).second->reset_undo();
  }

  for (histo_it = bg_histos.begin(); histo_it != bg_histos.end(); ++histo_it) {
    (*histo_it).second->reset_undo();
  }
}


/** Check if this color model uses histograms.
 * @return true
 */
bool
BayesColormapGenerator::has_histograms()
{
  return true;
}


/** Get histograms.
 * @return histograms
 */
std::map< hint_t, Histogram * > *
BayesColormapGenerator::get_histograms()
{
  return &histos;
}


/** Normalize histograms and compute overall background histogram. */
void
BayesColormapGenerator::normalize_histos()
{
  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it) {
    delete histo_it->second;
  }
  histos.clear();

  unsigned int fg_size = 0;
  unsigned int hval;
  float norm_factor;
  unsigned int norm_size = image_width * image_height;

  // generate normalized fg histograms
  for (histo_it = fg_histos.begin(); histo_it != fg_histos.end(); ++histo_it)
    {
      hint_t cur_object = histo_it->first;

      if ( bg_histos.find(cur_object) == bg_histos.end() ) {
	throw Exception("Corresponding background histogram is missing");
      }
      
      Histogram *fg = fg_histos[cur_object];
      Histogram *bg = bg_histos[cur_object];

      unsigned int fg_sum = fg->get_sum();
      unsigned int bg_sum = bg->get_sum();

      if ( (fg_sum + bg_sum) == 0 )
	{ continue; }

      Histogram *h  = new Histogram(lut_width, lut_height, lut_depth);
      histos[cur_object] = h;

      norm_factor = norm_size / float(fg_sum + bg_sum);

      for (unsigned int x = 0; x < lut_width; ++x) {
	for (unsigned int y = 0; y < lut_height; ++y) {
	  for (unsigned int z = 0; z < lut_depth; ++z) {
	    hval = (unsigned int) rint(float(fg->get_value(x, y, z)) * norm_factor);
	    h->set_value(x, y, z, hval);
	  }
	}
      }

      fg_size += h->get_sum();
    }

  // compute overall background histogram
  for (histo_it = bg_histos.begin(); histo_it != bg_histos.end(); ++ histo_it)
    {
      hint_t cur_object = histo_it->first;

      Histogram *fg = fg_histos[cur_object];
      Histogram *bg = bg_histos[cur_object];

      unsigned int fg_sum = fg->get_sum();
      unsigned int bg_sum = bg->get_sum();

      if ( (fg_sum + bg_sum) == 0 )
	{ continue; }

      Histogram *h = new Histogram(lut_width, lut_height, lut_depth);
      histos[H_BACKGROUND] = h;

      norm_factor = norm_size / float(fg_sum + bg_sum);

      for (unsigned int x = 0; x < lut_width; ++x) {
	for (unsigned int y = 0; y < lut_height; ++y) {
	  for (unsigned int z = 0; z < lut_depth; ++z) {
	    // normalize
	    hval = (unsigned int) rint( float(bg->get_value(x, y, z)) * norm_factor);
	    h->add(x, y, z, hval);

	    // substract all other normalized fg histograms
	    std::map< hint_t, Histogram * >::iterator hit;
	    for (hit = histos.begin(); hit != histos.end(); ++hit) {
	      if (hit->first == cur_object || hit->first == H_BACKGROUND)
		{ continue; }
	      
	      hval = hit->second->get_value(x, y, z);
	      h->sub(x, y, z, hval);
	    }
	  }
	}
      }
    }
  
  // normalize overall background histogram
  Histogram* h = histos[H_BACKGROUND];
  norm_factor = (norm_size - fg_size) / float( h->get_sum() );
  
  for (unsigned int x = 0; x < lut_width; ++x) {
    for (unsigned int y = 0; y < lut_height; ++y) {
      for (unsigned int z = 0; z < lut_depth; ++z) {
 	hval = (unsigned int) rint( float(h->get_value(x, y, z)) * norm_factor );
 	h->set_value(x, y, z, hval);
      }
    }
  }  
}
