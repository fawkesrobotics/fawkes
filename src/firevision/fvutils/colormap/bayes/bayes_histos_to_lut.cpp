
/**************************************************************************
 *  bayes_histos_to_lut.cpp - This file implements a class
 *                            that takes color histograms of objects as input,
 *                            and, together with probabilities of objects,
 *                            generates all the values for a lookup-table
 *                            that maps from colors to objects
 *
 *  Generated: Mon Jun 27 14:16:52 2005
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

#include <fvutils/colormap/bayes/bayes_histos_to_lut.h>
#include <fvutils/statistical/histogram.h>
#include <fvutils/colormap/yuvcm.h>
#include <fvutils/colormap/cmfile.h>
#include <core/exception.h>

#include <fvutils/color/color_object_map.h>

#include <iostream>
#include <string>
#include <cstdlib>
#include <cstdio>

using namespace std;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class BayesHistosToLut <fvutils/colormap/bayes/bayes_histos_to_lut.h>
 * LUT generation by using Bayesian method on histograms.
 * Generates a YUV colormap.
 * @author Martin Herakles.
 * @author Tim Niemueller
 * @author Daniel Beck
 */

/** Constructor.
 * @param histos histograms
 * @param d depth of lookup table
 * @param object type of the foreground object
 * @param w the width of the lookup table (u-resolution)
 * @param h the height of the lookup table (v-resolution)
 */
BayesHistosToLut::BayesHistosToLut(std::map<hint_t, Histogram*> &histos,
				   unsigned int d, hint_t object, unsigned int w, unsigned int h)
  : histograms(histos)
{
  width  = w;
  height = h;
  depth  = d;

  fg_object  = object;
  //  histograms = histos;

  // no as shmem segment
  lut = new YuvColormap(depth, width, height);

  min_probability = 0.3;
  min_prob_ball = 0.0;
  min_prob_green = 0.0;
  min_prob_yellow = 0.0;
  min_prob_blue = 0.0;
  min_prob_white = 0.0;
  min_prob_black = 0.0;
}

/** Destructor. */
BayesHistosToLut::~BayesHistosToLut()
{
  delete lut;
}

/** Get name.
 * @return BayesHistosToLut
 */
string
BayesHistosToLut::getName()
{
  return string("BayesHistosToLut");
}

/** Get object probability.
 * @param object object
 * @return probability.
 */
float
BayesHistosToLut::getObjectProb(hint_t object)
{
  // These object probabilities should better be read from config file.

  if (fg_object == H_BALL) {
    /*
    switch (object) {
    case H_BALL:
    */
      return 0.2;
    /*
      break;
    case H_BACKGROUND:
      return 0.8;
      break;
    case H_ROBOT:
      return 0.0;
      break;
    case H_FIELD:
      return 0.0;
      break;
    case H_GOAL_BLUE:
      return 0.0;
      break;
    case H_GOAL_YELLOW:
      return 0.0;
      break;
    case H_LINE:
      return 0.0;
      break;
    case H_UNKNOWN:
      return 0.0;
      break;
    default:
      cout << "(BayesHistosToLut::getObjectProb): Invalid object." << endl;
      exit(-1);
      return 0.0f;
      break;
    }
    */
  } else {
    if ( object_probabilities.find(object) != object_probabilities.end() ) {
      return object_probabilities[object];
    } else {
      cout << "returning 0" << endl;
      return 0.f;
    }
  }
}

/** P(u, v| object).
 * Get a-priori probability.
 * @param u YUV U-value
 * @param v YUV V-value
 * @param object object.
 * @return probability
 */
float
BayesHistosToLut::getAPrioriProb(unsigned int u,
				 unsigned int v,
				 hint_t object)
{
  unsigned int sum = 0;
  for (unsigned int y = 0; y < depth; ++y) {
    sum += histograms[object]->get_value(u, v, y);
  }

  return ( float(sum) / float(numberOfOccurrences[object]) );
}

/** P(u, v| object).
 * Get a-priori probability.
 * @param y YUV Y-value
 * @param u YUV U-value
 * @param v YUV V-value
 * @param object object.
 * @return probability
 */
float
BayesHistosToLut::getAPrioriProb(unsigned int y,
				 unsigned int u,
				 unsigned int v,
				 hint_t object)
{
  return ( float(histograms[object]->get_value(u, v, y)) / float(numberOfOccurrences[object]) );
}

/** P(object| u, v).
 * Get a-posteriori probability.
 * @param object objcet
 * @param u YUV U-value
 * @param v YUV V-value
 */
float
BayesHistosToLut::getAPosterioriProb(hint_t object,
				     unsigned int u,
				     unsigned int v)
{
  /* calculate "nenner" for bayes-formula,
     i.e. sum up the probabilities P(u, v| object) * P(object)
     over all objects */
  float sumOfProbabilities = 0.0;
  map<hint_t, Histogram*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    sumOfProbabilities += ( getAPrioriProb(u, v, (hint_t)hit->first) * getObjectProb((hint_t)hit->first) );
  }

  if (sumOfProbabilities != 0) {
    return getAPrioriProb(u, v, object) * getObjectProb(object) / sumOfProbabilities;
  }
  else
    return 0;
}

/** P(object| u, v).
 * Get a-posteriori probability.
 * @param object objcet
 * @param y YUV Y-value
 * @param u YUV U-value
 * @param v YUV V-value
 */
float
BayesHistosToLut::getAPosterioriProb(hint_t object,
				     unsigned int y,
				     unsigned int u,
				     unsigned int v)
{
  /* calculate "nenner" for bayes-formula,
     i.e. sum up the probabilities P(u, v| object) * P(object)
     over all objects */
  float sumOfProbabilities = 0.0;
  map<hint_t, Histogram*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    sumOfProbabilities += ( getAPrioriProb(y, u, v, (hint_t)hit->first) * getObjectProb((hint_t)hit->first) );
  }

  if (sumOfProbabilities != 0) {
    return getAPrioriProb(y, u, v, object) * getObjectProb(object) / sumOfProbabilities;
  }
  else
    return 0;
}

/** Get most likely object.
 * @param u YUV U-value
 * @param v YUV V-value
 * @return most likely object for this color
 */
hint_t
BayesHistosToLut::getMostLikelyObject(unsigned int u,
				      unsigned int v)
{
  // TODO sum over all y-values

  hint_t mostLikelyObject = H_UNKNOWN;
  float probOfMostLikelyObject = 0.0;
  map<hint_t, Histogram*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    float tmp = getAPosterioriProb((hint_t)hit->first, u, v);

    if (tmp > probOfMostLikelyObject) {
      probOfMostLikelyObject = tmp;
      mostLikelyObject = (hint_t)hit->first;
    }
  }

  if (probOfMostLikelyObject > min_probability) {
    return mostLikelyObject;
  }
  else {
    return H_UNKNOWN;
  }
}

/** Get most likely object.
 * @param y YUV Y-value
 * @param u YUV U-value
 * @param v YUV V-value
 * @return most likely object for this color
 */
hint_t
BayesHistosToLut::getMostLikelyObject(unsigned int y,
				      unsigned int u,
				      unsigned int v)
{
  hint_t mostLikelyObject = H_UNKNOWN;
  float probOfMostLikelyObject = 0.0;
  map<hint_t, Histogram*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    float tmp = getAPosterioriProb((hint_t)hit->first, y, u, v);

    if (tmp > probOfMostLikelyObject) {
      probOfMostLikelyObject = tmp;
      mostLikelyObject = (hint_t)hit->first;
    }
  }

  if (probOfMostLikelyObject > min_probability) {
    return mostLikelyObject;
  }
  else {
    return H_UNKNOWN;
  }
}

/** Calculate all LUT colors. */
void
BayesHistosToLut::calculateLutAllColors()
{
  // for each histogram, sum up all of its entries
  //  numberOfOccurrences.resize( histograms.size() );
  map<hint_t, Histogram*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    unsigned int total = 0;
    for (unsigned int v = 0; v < height; ++v) {
      for (unsigned int u = 0; u < width; ++u) {
	for (unsigned int y = 0; y < depth; ++y) {
	  unsigned int tmp = ((Histogram*)(hit->second))->get_value(u, v, y);
	  if (tmp > 0)
	    total += tmp;
	}
      }
    }
    numberOfOccurrences[ (hint_t)hit->first ] = total;
  }

  /*
  cout << "histo-BALL : " << numberOfOccurrences[0] << " counts." << endl
       << "histo-GREEN: " << numberOfOccurrences[3] << " counts." << endl
       << "histo-BLUE : " << numberOfOccurrences[5] << " counts." << endl;
  */

  // for each color, mark it (in lut) as the color
  // that has the highest probability (among all histograms)
  hint_t   color_with_highest_prob;
  float    highest_prob;
  float    current_prob;
  for (unsigned int y = 0; y < depth; ++y) {
    unsigned int y_index = y * lut->deepness() / lut->depth();
    for (unsigned int v = 0; v < height; ++v) {
      for (unsigned int u = 0; u < width; ++u) {

	// find most probable color for (u, v)
	highest_prob = 0.0;
	color_with_highest_prob = H_UNKNOWN; // ...maybe it is better to have default = H_BACKGROUND...
	map<hint_t, Histogram*>::iterator hit;
	for (hit = histograms.begin(); hit != histograms.end(); hit++) {
	  // if current histogram is not empty...
	  if (numberOfOccurrences[ (hint_t)hit->first ] > 0) {
	    current_prob = float( hit->second->get_value(u, v, y) ) / float( numberOfOccurrences[ hit->first ] );
	    // if current histogram has higher probability for color (u, v),
	    // _and_ is above min_prob-threshold...
	    if ( current_prob > highest_prob &&
		 current_prob > min_probability ) {
	      // ...update color information
	      highest_prob = current_prob;
	      color_with_highest_prob = hit->first;
	    }
	  }
	}

	// set lut value for color (u, v) to most probable color
	lut->set(y_index, u, v, ColorObjectMap::get_instance().get(color_with_highest_prob));
      }
    }
  }

}


/** Calculate LUT values.
 * @param penalty if true, non-ball colors are penalized
 */
void
BayesHistosToLut::calculateLutValues( bool penalty )
{

  unsigned int old_undo = 0;

  if ( penalty ) {
    // We penalize all values, that have NOT been classified as ball
    Histogram *histo_fg = histograms[fg_object];
    Histogram *histo_bg = histograms[H_BACKGROUND];

    if ( histo_bg->get_num_undos() < 2 ) {
      // No undo available for us
      cout << "Histogram::calculateLutValues: There are not enough undos possible for background histogram, not penalizing" << endl;
    } else {
      unsigned int bg_median  = histo_bg->get_median();
      unsigned int bg_average = histo_bg->get_average();
       unsigned int bg_val = 0;

      old_undo = histo_bg->switch_undo( 1 );

      cout << "Histogram: Setting low bg vals to median. median=" << bg_median
	   << "  avg=" << bg_average << endl;

      for (unsigned int v = 0; v < height; ++v) {
	for (unsigned int u = 0; u < width; ++u) {
	  for (unsigned int y = 0; y < depth; ++y) {

	    if ( histo_fg->get_value(u, v, y) == 0 ) {
	      bg_val = histo_bg->get_value(u, v, y);
	      if (bg_val < bg_average) {
		histo_bg->set_value(u, v, y, bg_average);
	      }
	    }
	  }
	}
      }
    }
  }

  /* count for each object
     how many non-zero values its histogram has in total */
  //  numberOfOccurrences.resize(histograms.size());

  map<hint_t, Histogram*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    unsigned int total = 0;
    for (unsigned int y = 0; y < depth; ++y) {
      for (unsigned int v = 0; v < height; ++v) {
	for (unsigned int u = 0; u < width; ++u) {
	  unsigned int tmp = hit->second->get_value(u, v, y);
	  if (tmp > 0)
	    total += tmp;
	}
      }
    }
    numberOfOccurrences[hit->first] = total;
    cout << "[" << hit->first << "]: " << numberOfOccurrences[hit->first] << " occurences" << endl;
  }

  unsigned int total_count = 0;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    total_count += hit->second->get_sum();
  }
  //  cout << "Total count: " << total_count << endl;

  // Calculate overall object probabilities
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    object_probabilities[hit->first] = (float)hit->second->get_sum() / (float)total_count;

    //     cout << "Setting a-priori probability for histogram " << hit->first << " to "
    // 	 << object_probabilities[hit->first] << endl;
  }


  unsigned int count_ball       = 0;
  unsigned int count_field      = 0;
  unsigned int count_line       = 0;
  unsigned int count_robot      = 0;
  unsigned int count_background = 0;
  unsigned int count_goal       = 0;
  unsigned int count_unknown    = 0;

  lut->reset();

  for (unsigned int y = 0; y < depth; ++y) {
    unsigned int y_index = y * lut->deepness() / lut->depth();
    for (unsigned int u = 0; u < width; ++u) {
      unsigned int u_index = u * lut->deepness() / lut->width();
      for (unsigned int v = 0; v < height; ++v) {
        unsigned int v_index = v * lut->deepness() / lut->height();
	hint_t mostLikelyObject = getMostLikelyObject(y, u, v);

	switch(mostLikelyObject) {
	case H_BALL:
	  count_ball++;
	  break;
	case H_BACKGROUND:
	  count_background++;
	  break;
	case H_ROBOT:
	case H_ROBOT_OPP:
	  count_robot++;
	  break;
	case H_FIELD:
	  count_field++;
	  break;
	case H_LINE:
	  count_line++;
	  break;
	case H_GOAL_YELLOW:
	case H_GOAL_BLUE:
	  count_goal++;
	  break;
	case H_UNKNOWN:
	  count_unknown++;
	  break;
	default:
	  cout << "(BayesHistosToLut::calculateLutValues(): Invalid object." << endl;
	  throw fawkes::Exception("BayesHistosToLut::calculateLutValues(): Invalid object.");
	}
        lut->set(y_index, u_index, v_index, ColorObjectMap::get_instance().get(mostLikelyObject));
      }
    }
  }

	printf("d/w/h: %u/%u/%u  ball: %d  field: %d  line: %d  robot: %d  goal: %d  background: %d  unknown: %d\n",
	       depth, width, height, count_ball, count_field, count_line,
	       count_robot, count_goal, count_background, count_unknown);

  if ( penalty ) {
    Histogram *histo_bg   = histograms[H_BACKGROUND];
    if ( histo_bg->get_num_undos() >= 2 ) {
      histo_bg->undo();
      histo_bg->switch_undo( old_undo );
    }
  }


  /*
  // for testing: output ball colors
  cout << " ============" << endl;
  for (unsigned int v = 0; v < height; v++) {
    for (unsigned int u = 0; u < width; u++) {
      if (lut->determine(128, u, v) == BACKGROUND)
	cout << "lut says that (" << u << ", " << v << ") is background color." << endl;
    }
  }
  cout << "===============" << endl;
  */
}

/** Save LUT to file.
 * @param file file name
 */
void
BayesHistosToLut::saveLut(char *file)
{
  ColormapFile cmf;
  cmf.add_colormap(lut);
  cmf.write(file);
}

/** Save LUT to file.
 * @param filename file name
 */
void
BayesHistosToLut::save(std::string filename)
{
  ColormapFile cmf;
  cmf.add_colormap(lut);
  cmf.write(filename.c_str());
}


/** Set min probability.
 * @param min_prob minimum probability
 */
void
BayesHistosToLut::setMinProbability( float min_prob )
{
  min_probability = min_prob;
}


/** Set min probability for color.
 * @param min_prob minimum probability
 * @param hint color hint
 */
void
BayesHistosToLut::setMinProbForColor( float min_prob, hint_t hint ) {
  switch( hint ) {
  case H_BALL:
    min_prob_ball = min_prob;
    break;
  case H_FIELD:
    min_prob_green = min_prob;
    break;
  case H_GOAL_YELLOW:
    min_prob_yellow = min_prob;
    break;
  case H_GOAL_BLUE:
    min_prob_blue = min_prob;
    break;
  case H_LINE:
    min_prob_white = min_prob;
    break;
  case H_ROBOT:
    min_prob_black = min_prob;
    break;
  default:
    /**/
    break;
  }
}


/** Get generated color model.
 * @return generated color model
 */
YuvColormap *
BayesHistosToLut::get_colormap()
{
  return lut;
}

} // end namespace firevision
