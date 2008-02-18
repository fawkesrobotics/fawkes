
/***************************************************************************
 *  histogram.cpp - Implementation of the 2D histogram
 *
 *  Generated: Tue Jun 14 11:11:29 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
 *
 ****************************************************************************/

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

#include <fvutils/statistical/histogram.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <cstring>

using namespace std;


/** @class Histogram2D <fvutils/statistical/histogram.h>
 * 2D Histogram.
 * Histrogram with 2D coordinates for buckets.
 */

/** Constructor.
 * @param width width of histogram plane
 * @param height height of histogram plane
 * @param num_undos number of possible undos
 */
Histogram2D::Histogram2D(unsigned int width, unsigned int height, unsigned int num_undos) {
  if ( (width == 0) || (height == 0) ) {
    throw new string("Width or height is zero.");
  }

  this->width        = width;
  this->height       = height;
  this->undo_num     = num_undos;
  this->undo_current = 0;

  histogram    = (unsigned int *)malloc(width * height * sizeof(unsigned int));
  undo_overlay = (unsigned int **)malloc(num_undos * sizeof(unsigned int *));
  for (unsigned int i = 0; i < num_undos; ++i) {
    undo_overlay[i] = (unsigned int *)malloc(width * height * sizeof(unsigned int));
  }

  undo_num_vals = (unsigned int *)malloc(num_undos * sizeof(unsigned int));

  reset();

}


/** Destructor. */
Histogram2D::~Histogram2D()
{
  free(histogram);
  for (unsigned int i = 0; i < undo_num; ++i) {
    free(undo_overlay[i]);
  }
  free(undo_overlay);
}


/** Add point.
 * @param p point
 */
void
Histogram2D::operator+=(point_t *p)
{
  histogram[p->y * width + p->x ] += 1;
  undo_overlay[undo_current][p->y * width + p->x ] += 1;
  ++number_of_values;
  undo_num_vals[undo_current] += 1;
}


/** Add point.
 * @param p point
 */
void
Histogram2D::operator+=(point_t p)
{
  histogram[ p.y * width + p.x ] += 1;
  undo_overlay[undo_current][ p.y * width + p.x ] += 1;
  ++number_of_values;
  undo_num_vals[undo_current] += 1;
}


/** Get histogram data buffer.
 * @return histogram
 */
unsigned int *
Histogram2D::getHistogram()
{
  return histogram;
}


/** Get value from histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 */
unsigned int
Histogram2D::getValue(unsigned int x, unsigned int y)
{
  return histogram[y * width + x ];
}


/** Set value in histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 * @param value value
 */
void
Histogram2D::setValue(unsigned int x, unsigned int y, unsigned int value) 
{
  unsigned int oldValue = histogram[ y * width + x ];
  histogram[ y * width + x ] = value;
  number_of_values += value - oldValue;
  if ( value > oldValue ) {
    // The value got incremented, add to overlay
    undo_overlay[undo_current][y * width + x] += value - oldValue;
  } else {
    if ( (oldValue - value) < undo_overlay[undo_current][y * width + x] ) {
      undo_overlay[undo_current][y * width + x] -= (oldValue - value);
    } else {
      // We cannot handle values smaller than the original value, this is
      // due to choosing unsigned int as datatype, right now this should suffice
      undo_overlay[undo_current][y * width + x] = 0;
    }
  }
}


/** Reset histogram. */
void
Histogram2D::reset()
{
  memset(histogram, 0, (width * height * sizeof(unsigned int)));
  number_of_values = 0;
  for (unsigned int i = 0; i < undo_num; ++i) {
    switchUndo( i );
    resetUndo();
  }
  switchUndo( 0 );
}


/** Print to stream.
 * @param s stream
 */
void
Histogram2D::printToStream(std::ostream &s)
{
  for (unsigned int v = 0; v < height; ++v) {
    for (unsigned int u = 0; u < width; ++u) {
      cout << histogram[v * width + u ] << " ";
    }
    cout << endl;
  }
  cout << endl;
}


/** Save to file.
 * @param filename file name to save to
 * @param formatted_output one value per line
 */
void
Histogram2D::save(const char *filename, bool formatted_output) {
  std::ofstream file;
  file.open( filename );

  for (unsigned int v = 0; v < height; ++v) {
    for (unsigned int u = 0; u < width; ++u) {
      if (formatted_output) {
	if (0 != histogram[v * width + u ]) {
	  file << u << " " << v << " " << histogram[v * width + u ] << endl;
	}
      } else {
	file << histogram[v * width + u ] << " ";
      }
    }
  }

  file.close();
  cout << "Histogram2D: Saved histogram in file \"" << filename << "\"." << endl;
}


/** Load from file.
 * @param filename file name to read from
 */
bool
Histogram2D::load(const char *filename) {
  std::ifstream file;
  file.open( filename );

  if (!file) {
    cout << "Histogram2D: Could not load histogram from file\"" << filename << "\"." << endl;
    return false;
  }
  else {
    for (unsigned int v = 0; v < height; ++v) {
      for (unsigned int u = 0; u < width; ++u) {
	file >> histogram[v * width + u ];
      }
    }
    file.close();
    cout << "Histogram2D: Loaded histogram from file \"" << filename << "\"." << endl;
    return true;
  }
}


/** Reset undo. */
void
Histogram2D::resetUndo()
{
  memset(undo_overlay[undo_current], 0, (width * height * sizeof(unsigned int)));
  undo_num_vals[undo_current] = 0;
}


/** Undo. */
void
Histogram2D::undo()
{
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      histogram[ y * width + x ] -= undo_overlay[undo_current][ y * width + x ];
    }
  }
  number_of_values -= undo_num_vals[undo_current];
  resetUndo();
}


/** Switch undo to another undo buffer.
 * @param undo_id switch to buffer with this ID
 * @return returns current undo buffer ID
 */
unsigned int
Histogram2D::switchUndo( unsigned int undo_id )
{
  unsigned int undo_last = undo_current;

  if (undo_id >= undo_num) {
    cout << "Histogram2D::resetUndo: ID out of range" << endl;
  } else {
    undo_current = undo_id;
  }

  return undo_last;
}


/** Get number of undos.
 * @return number of undos
 */
unsigned int
Histogram2D::getNumUndos()
{
  return undo_num;
}


/** Get median of all values.
 * @return median
 */
unsigned int
Histogram2D::getMedian()
{
  vector< unsigned int > *values = new vector< unsigned int >( width * height );

  values->clear();
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if ( histogram[y * width + x] > 0 ) {
        values->push_back( histogram[y * width + x ] );
      }
    }
  }

  sort(values->begin(), values->end());

  unsigned int median = values->at( values->size() / 2 );

  delete values;

  return median;
}


/** Get average of all values.
 * @return average
 */
unsigned int
Histogram2D::getAverage()
{
  unsigned int sum = 0;
  unsigned int num = 0;
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if ( histogram[y * width + x ] ) {
        sum += histogram[y * width + x ];
        num++;
      }
    }
  }
  
  return (sum / num);
}


/** Get sum of all values.
 * @return sum of values
 */
unsigned int
Histogram2D::getSum() const
{
  unsigned int sum = 0;
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if ( histogram[y * width + x ] ) {
        sum += histogram[y * width + x ];
      }
    }
  }

  return sum;
}
