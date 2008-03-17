
/**************************************************************************
 *  bayes_generator.cpp - generator for lookuptables using a bayesian method
 *
 *  Generated: Wed Mar 01 14:14:41 2006
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

#include <models/color/bayes/bayes_generator.h>

#include <fvutils/color/yuv.h>
#include <fvutils/statistical/histogram.h>
#include <models/color/lookuptable.h>
#include <models/color/bayes/bayes_histos_to_lut.h>
#include <core/exception.h>

#include <cmath>

using namespace std;

/** @class BayesColorLutGenerator <models/color/bayes/bayes_generator.h>
 * Color LUT Generator using Bayes method.
 * @author Tim Niemueller
 * @author Daniel Beck
 */

/** Constructor. 
 * @param lut_width the width of the lookup table
 * @param lut_height the height of the lookup table
 * @param lut_depth the depth of the lookup table
 * @param fg_object the type of a foreground object
 */
BayesColorLutGenerator::BayesColorLutGenerator( unsigned int lut_width,
						unsigned int lut_height,
						unsigned int lut_depth,
						hint_t fg_object)
{
  this->lut_width  = lut_width;
  this->lut_height = lut_height;
  this->lut_depth  = lut_depth;

  set_fg_object(fg_object);

  histos.clear();
  fg_histos.clear();
  bg_histos.clear();

  histos[H_BACKGROUND] = new Histogram(lut_width, lut_height, lut_depth, 2);

  image_width = image_height = 0;
  selection_mask = 0;

  bhtl = new BayesHistosToLut(histos, lut_width, lut_height, lut_depth, fg_object);
  cm = bhtl->getColorModel();
}


/** Destructor. */
BayesColorLutGenerator::~BayesColorLutGenerator()
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
BayesColorLutGenerator::set_fg_object(hint_t object)
{
  if (H_UNKNOWN == object)
    { return; }

  if ( fg_histos.find(object) == fg_histos.end() ) {
    //    printf("Adding histos for object type %d\n", object);
    fg_histos[object] = new Histogram(lut_width, lut_height, lut_depth);
    bg_histos[object] = new Histogram(lut_width, lut_height, lut_depth, 2);
    histos[object] = new Histogram(lut_width, lut_height, lut_depth);
  }

  fg_object = object;
  //  printf("Active object type is %d\n", fg_object);
}


/** Set buffer.
 * @param buffer image buffer
 * @param width image width
 * @param height image height
 */
void
BayesColorLutGenerator::set_buffer(unsigned char *buffer,
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
ColorModelLookupTable *
BayesColorLutGenerator::get_current()
{
  return cm;
}


/** Check if pixel is in region.
 * @param x image x coordinate
 * @param y image y coordinate
 * @return true if pixel is in region, false otherwise
 */
bool
BayesColorLutGenerator::is_in_region(unsigned int x, unsigned int y) 
{
  return selection_mask[image_width * y + x];
}


/** Set selection.
 * @param region selected region.
 */
void
BayesColorLutGenerator::set_selection(vector< rectangle_t > region)
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
BayesColorLutGenerator::set_min_probability(float min_prob)
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
BayesColorLutGenerator::calc()
{
  normalize_histos();
  bhtl->calculateLutValues( false /* no penalty*/ );
}


/** Undo last inclusion. */
void
BayesColorLutGenerator::undo()
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
BayesColorLutGenerator::reset()
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
BayesColorLutGenerator::reset_undo()
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
BayesColorLutGenerator::has_histograms()
{
  return true;
}


/** Get histograms.
 * @return histograms
 */
std::map< hint_t, Histogram * > *
BayesColorLutGenerator::get_histograms()
{
  return &histos;
}


/** Normalize histograms and compute overall background histogram. */
void
BayesColorLutGenerator::normalize_histos()
{
  for (histo_it = histos.begin(); histo_it != histos.end(); ++histo_it)
    {
      histo_it->second->reset();
    }

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
      Histogram *h  = histos[cur_object];

      norm_factor = norm_size / float(fg->get_sum() + bg->get_sum());

      for (unsigned int x = 0; x < lut_width; ++x) {
	for (unsigned int y = 0; y < lut_height; ++y) {
	  for (unsigned int z = 0; z < lut_depth; ++z) {
	    hval = (unsigned int) rint(float(fg->get_value(x, y, z)) * norm_factor);
	    h->set_value(x, y, z, hval);
	  }
	}
      }

      fg_size += fg->get_sum();

      //      printf("[%d] normalized size=%d\n", cur_object, fg->get_sum());
    }

  // compute overall background histogram
  for (histo_it = bg_histos.begin(); histo_it != bg_histos.end(); ++ histo_it)
    {
      hint_t cur_object = histo_it->first;

      Histogram *fg  = fg_histos[cur_object];
      Histogram *bg  = bg_histos[cur_object];
      Histogram *h   = histos[H_BACKGROUND];

      norm_factor = norm_size / float(fg->get_sum() + bg->get_sum());

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
  
  //  printf("overall fg size=%d  bg size=%d\n", fg_size, histos[H_BACKGROUND]->get_sum() );
	    
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
  
  //  printf("overall fg size=%d  bg size=%d\n", fg_size, histos[H_BACKGROUND]->get_sum() );
}
