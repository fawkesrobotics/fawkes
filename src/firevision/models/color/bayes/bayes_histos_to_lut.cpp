
/**************************************************************************
 *  bayes_histos_to_lut.cpp - This file implements a class
 *                            that takes color histograms of objects as input,
 *                            and, together with probabilities of objects,
 *                            generates all the values for a lookup-table
 *                            that maps from colors to objects
 *
 *  Generated: Mon Jun 27 14:16:52 2005
 *  Copyright  2005  Martin Heracles
 *                   Tim Niemueller [www.niemueller.de]
 *             2007  Daniel Beck
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

#include <models/color/bayes/bayes_histos_to_lut.h>
#include <fvutils/statistical/histogram.h>
#include <models/color/lookuptable.h>

// include <utils/utils.h>

#include <iostream>
#include <string>

using namespace std;

/** @class BayesHistosToLut <models/color/bayes/bayes_histos_to_lut.h>
 * LUT generation by using Bayesian method on histograms.
 * @author Martin Herakles.
 * @author Tim Niemueller
 * @author Daniel Beck
 */

/** Constructor.
 * @param histos histograms
 * @param w width of lookup table
 * @param h height of lookup table
 * @param object type of the foreground object
 */
BayesHistosToLut::BayesHistosToLut(map<hint_t, Histogram2D*> histos,
				   unsigned int w, 
				   unsigned int h,
				   hint_t object)
{
  width = w;
  height = h;
  fg_object = object;
  histograms = histos;

  // no as shmem segment
  lut = new ColorModelLookupTable(width, height);

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
  return ( float(histograms[object]->getValue(u, v)) / float(numberOfOccurrences[object]) );
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
  map<hint_t, Histogram2D*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    sumOfProbabilities += ( getAPrioriProb(u, v, (hint_t)hit->first) * getObjectProb((hint_t)hit->first) );
  }

  if (sumOfProbabilities != 0) {
    return getAPrioriProb(u, v, object) * getObjectProb(object) / sumOfProbabilities;
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
  hint_t mostLikelyObject = H_UNKNOWN;
  float probOfMostLikelyObject = 0.0;
  map<hint_t, Histogram2D*>::iterator hit;
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


/** Calculate all LUT colors. */
void
BayesHistosToLut::calculateLutAllColors() 
{
  // for each histogram, sum up all of its entries
  //  numberOfOccurrences.resize( histograms.size() );
  map<hint_t, Histogram2D*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    unsigned int total = 0;
    for (unsigned int v = 0; v < height; ++v) {
      for (unsigned int u = 0; u < width; ++u) {
	unsigned int tmp = ((Histogram2D*)(hit->second))->getValue(u, v);
	if (tmp > 0)
	  total += tmp;
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
  for (unsigned int v = 0; v < height; v++) {
    for (unsigned int u = 0; u < width; u++) {

      // find most probable color for (u, v)
      highest_prob = 0.0;
      color_with_highest_prob = H_UNKNOWN; // ...maybe it is better to have default = H_BACKGROUND...
      map<hint_t, Histogram2D*>::iterator hit;
      for (hit = histograms.begin(); hit != histograms.end(); hit++) {
	// if current histogram is not empty...
	if (numberOfOccurrences[ (hint_t)hit->first ] > 0) {
	  current_prob = float( hit->second->getValue(u, v) ) / float( numberOfOccurrences[ hit->first ] );
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
      switch( color_with_highest_prob ) {
      case H_BALL:
	lut->set(128, u, v, C_ORANGE);
	break;
      case H_BACKGROUND:
	lut->set(128, u, v, C_BACKGROUND);
	break;
      case H_ROBOT:
	lut->set(128, u, v, C_BLACK);
	break;
      case H_FIELD:
	lut->set(128, u, v, C_GREEN);
	break;
      case H_GOAL_BLUE:
	lut->set(128, u, v, C_BLUE);
	break;
      case H_GOAL_YELLOW:
	lut->set(128, u, v, C_YELLOW);
	break;
      case H_LINE:
	lut->set(128, u, v, C_WHITE);
	break;
      case H_UNKNOWN:
	lut->set(128, u, v, C_OTHER);
	break;
      default:
	cout << "(BayesHistosToLut::calculateLutAllColors): Invalid object." << endl;
	exit(-1);
	break;
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
    Histogram2D *histo_fg = histograms[fg_object];
    Histogram2D *histo_bg = histograms[H_BACKGROUND];

    if ( histo_bg->getNumUndos() < 2 ) {
      // No undo available for us
      cout << "Histogram2D::calculateLutValues: There are not enough undos possible for background histogram, not penalizing" << endl;
    } else {

      unsigned int bg_median  = histo_bg->getMedian();
      unsigned int bg_average = histo_bg->getAverage();
      unsigned int bg_val = 0;

      old_undo = histo_bg->switchUndo( 1 );

      cout << "Histogram2D: Setting low bg vals to median. median=" << bg_median
	   << "  avg=" << bg_average << endl;

      for (unsigned int v = 0; v < height; ++v) {
	for (unsigned int u = 0; u < width; ++u) {
	  if ( histo_fg->getValue( u, v ) == 0 ) {
	    bg_val = histo_bg->getValue( u, v );
	    if (bg_val < bg_average) {
	      histo_bg->setValue( u, v, bg_average );
	    }
	  }
	}
      }
    }
  }

  /* count for each object 
     how many non-zero values its histogram has in total */
  //  numberOfOccurrences.resize(histograms.size());

  map<hint_t, Histogram2D*>::iterator hit;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    unsigned int total = 0;
    for (unsigned int v = 0; v < height; ++v) {
      for (unsigned int u = 0; u < width; ++u) {
	unsigned int tmp = hit->second->getValue(u, v);
	if (tmp > 0)
	  total += tmp;
      }
    }
    numberOfOccurrences[hit->first] = total;
    cout << "[" << hit->first << "]: " << numberOfOccurrences[hit->first] << " occurences" << endl;
  }

  unsigned int total_count = 0;
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    total_count += hit->second->getSum();
  }
  //  cout << "Total count: " << total_count << endl;

  // Calculate overall object probabilities
  for (hit = histograms.begin(); hit != histograms.end(); hit++) {
    object_probabilities[hit->first] = (float)hit->second->getSum() / (float)total_count;

    //    cout << "Setting probability for histogram " << hit->first << " to "
    //	 << object_probabilities[hit->first] << endl;
  }


  for (unsigned int v = 0; v < height; v++) {
    for (unsigned int u = 0; u < width; u++) {
      hint_t mostLikelyObject = getMostLikelyObject(u, v);

      switch(mostLikelyObject) {
      case H_BALL:
	lut->set(128, u, v, C_ORANGE);
	break;
      case H_BACKGROUND:
	lut->set(128, u, v, C_BACKGROUND);
	break;
      case H_FIELD:
	lut->set(128, u, v, C_GREEN);
	break;
      case H_GOAL_BLUE:
	lut->set(128, u, v, C_BLUE);
	break;
      case H_GOAL_YELLOW:
	lut->set(128, u, v, C_YELLOW);
	break;
      case H_UNKNOWN:
	lut->set(128, u, v, C_OTHER);
	break;
      default:
	cout << "(BayesHistosToLut::calculateLutValues): Invalid object." << endl;
	exit(-1);
	break;
      }
    }
  }

  if ( penalty ) {
    Histogram2D *histo_bg   = histograms.at( H_BACKGROUND );
    if ( histo_bg->getNumUndos() >= 2 ) {
      histo_bg->undo();
      histo_bg->switchUndo( old_undo );
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
  lut->save(file);
}

/** Save LUT to file.
 * @param filename file name
 */
void
BayesHistosToLut::save(string filename)
{
  lut->save( (char *)filename.c_str() );
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
ColorModelLookupTable *
BayesHistosToLut::getColorModel()
{
  return lut;
}
