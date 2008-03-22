
/***************************************************************************
 *  histogram.cpp - Implementation of the histogram
 *
 *  Generated: Tue Jun 14 11:11:29 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *             2008  Daniel Beck
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
#include <core/exceptions/software.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <cstring>

using namespace std;


/** @class Histogram <fvutils/statistical/histogram.h>
 * Histogram.
 * Histrogram with 2D or 3D coordinates for buckets.
 */

/** Constructor.
 * @param width width of histogram plane
 * @param height height of histogram plane
 * @param depth depth of the histogram
 * @param num_undos number of possible undos
 */
Histogram::Histogram(unsigned int width, unsigned int height, 
		     unsigned int depth, unsigned int num_undos) 
{
  if ( (width == 0) || (height == 0) || (depth == 0) ) {
    throw Exception("Width or height or depth is zero.");
  }

  this->width        = width;
  this->height       = height;
  this->depth        = depth;
  this->undo_num     = num_undos;
  this->undo_current = 0;

  if (depth == 1) {
    dimension = 2;
  } else {
    dimension = 3;
  }

  histogram_size = width * height * depth * sizeof(unsigned int);

  histogram    = (unsigned int *)malloc(histogram_size);
  undo_overlay = (unsigned int **)malloc(num_undos * sizeof(unsigned int *));
  for (unsigned int i = 0; i < num_undos; ++i) {
    undo_overlay[i] = (unsigned int *)malloc(histogram_size);
  }

  undo_num_vals = (unsigned int *)malloc(num_undos * sizeof(unsigned int));

  reset();
}


/** Destructor. */
Histogram::~Histogram()
{
  free(histogram);
  for (unsigned int i = 0; i < undo_num; ++i) {
    free(undo_overlay[i]);
  }
  free(undo_overlay);
  free(undo_num_vals);
}


/** Add point.
 * @param p point
 */
void
Histogram::operator+=(point_t *p)
{
  if ( dimension != 2 ) {
    throw Exception("Trying to add 2-dim data to 3-dim histogram");
  }

  if (p->x >= width || p->y >= height) {
    throw OutOfBoundsException("Point lies outside of histogram range");
  }

  unsigned int index = p->y * width + p->x;
  histogram[index] += 1;
  undo_overlay[undo_current][index] += 1;
  ++number_of_values;
  undo_num_vals[undo_current] += 1;
}


/** Add point.
 * @param p point
 */
void
Histogram::operator+=(point_t p)
{
  if ( dimension != 2 ) {
    throw Exception("Trying to add 2-dim data to 3-dim histogram");
  }

  if (p.x >= width || p.y >= height) {
    throw OutOfBoundsException("Point lies outside of histogram range");
  }

  unsigned int index = p.y * width + p.x;
  histogram[index] += 1;
  undo_overlay[undo_current][index] += 1;
  ++number_of_values;
  undo_num_vals[undo_current] += 1;
}


/** Get histogram data buffer.
 * @return histogram
 */
unsigned int *
Histogram::get_histogram()
{
  return histogram;
}


/** Get value from histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 */
unsigned int
Histogram::get_value(unsigned int x, unsigned int y)
{
  if ( dimension != 2 ) {
    throw Exception("Histogram::get_value: trying to access 2-dim value in 3-dim histogram");
  }

  if ( x >= width || y >= height) {
    throw Exception("Histogram::get_value: x=%d, or y=%d", x, y);
  }

  return histogram[y * width + x];
}


/** Get value from histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 * @param z z coordinate in the histogram
 */
unsigned int
Histogram::get_value(unsigned int x, unsigned int y, unsigned int z)
{
  if ( dimension != 3 ) {
    throw Exception("Histogram::get_value: trying to access 3-dim value in 2-dim histogram");
  }

  if ( x >= width || y >= height || z >= depth) {
    throw Exception("Histogram::get_value: x=%d, y=%d, or z=%d", x, y, z);
  }

  return histogram[z * width * height + y * width + x];
}


/** Set value in histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 * @param value value
 */
void
Histogram::set_value(unsigned int x, unsigned int y, unsigned int value) 
{
  if ( x >= width || y >= height) {
    throw OutOfBoundsException("Histogram::set_value(x, y, v): width, height, or depth out of range");
  }

  if ( dimension != 2 ) {
    printf("Histogram::set_value(x, y, val): this is a 3-dimensional histogram");
    return;
  }

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


/** Set value in histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 * @param z z coordinate in the histogram
 * @param value value
 */
void
Histogram::set_value(unsigned int x, unsigned int y, unsigned int z, unsigned int value) 
{  
  if ( x >= width || y >= height || z >= depth) {
    throw OutOfBoundsException("Histogram::set_value(x, y, z, v): width, height, or depth out of range");
  }

  unsigned int index = z * width * height + y * width + x;
  unsigned int oldValue = histogram[index];
  histogram[index] = value;
  number_of_values += value - oldValue;
  if ( value > oldValue ) {
    // The value got incremented, add to overlay
    undo_overlay[undo_current][index] += value - oldValue;
  } else {
    if ( (oldValue - value) < undo_overlay[undo_current][index] ) {
      undo_overlay[undo_current][index] -= (oldValue - value);
    } else {
      // We cannot handle values smaller than the original value, this is
      // due to choosing unsigned int as datatype, right now this should suffice
      undo_overlay[undo_current][index] = 0;
    }
  }
}


/** Increase the value of the histogram at given position.
 * @param x x coordinate in the histogram
 * @param y y coordinate in the histogram
 * @param z z coordinate in the histogram
 */
void
Histogram::inc_value(unsigned int x, unsigned int y, unsigned int z)
{
  if ( x >= width || y >= height || z >= depth) {
    throw OutOfBoundsException("Histogram::inc_value(): width, height, or depth out of range");
  }
  
  unsigned int index = z * width * height + y * width + x;
  histogram[index]++;
  undo_overlay[undo_current][index] = 1;
}


/** Add value to value in histogram at given location.
 * @param x x coordinate in histogram
 * @param y y coordinate in histogram
 * @param z z coordinate in histogram
 * @param value the value to add
 */
void
Histogram::add(unsigned int x, unsigned int y, unsigned int z, unsigned int value)
{
  if ( x >= width || y >= height || z >= depth) {
    throw OutOfBoundsException("Histogram::inc_value(): width, height, or depth out of range");
  }

  unsigned int cur_val = get_value(x, y, z);
  set_value(x, y, z, cur_val + value);
}


/** Substract value from value in histogram at given location.
 * @param x x coordinate in histogram
 * @param y y coordinate in histogram
 * @param z z coordinate in histogram
 * @param value the value to substract
 */
void
Histogram::sub(unsigned int x, unsigned int y, unsigned int z, unsigned int value)
{
  if ( x >= width || y >= height || z >= depth) {
    throw OutOfBoundsException("Histogram::inc_value(): width, height, or depth out of range");
  }

  unsigned int cur_val = get_value(x, y, z);
  if (value < cur_val) {
    set_value(x, y, z, cur_val - value);
  } else {
    set_value(x, y, z, 0);
  }
}


/** Reset histogram. */
void
Histogram::reset()
{
  memset(histogram, 0, histogram_size);
  number_of_values = 0;
  for (unsigned int i = 0; i < undo_num; ++i) {
    switch_undo( i );
    reset_undo();
  }
  switch_undo( 0 );
}


/** Print to stream.
 * @param s stream
 */
void
Histogram::print_to_stream(std::ostream &s)
{
  for (unsigned int z = 0; z < depth; ++z) {
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
	cout << histogram[z * width * height + y * width + x] << " ";
      }
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
Histogram::save(const char *filename, bool formatted_output) {
  std::ofstream file;
  file.open( filename );

  for (unsigned int z = 0; z < depth; ++z) {
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
	if (formatted_output) {
	  if (0 != histogram[z * width * height + y * width + x]) {
	    file << z << " " << y << " " << x << " " << histogram[z * width * height + y * width + x] << endl;
	  }
	} else {
	  file << histogram[z * width * height + y * width + x] << " ";
	}
      }
    }
  }

  file.close();
  cout << "Histogram: Saved histogram in file \"" << filename << "\"." << endl;
}


/** Load from file.
 * @param filename file name to read from
 */
bool
Histogram::load(const char *filename) {
  std::ifstream file;
  file.open( filename );

  if (!file) {
    cout << "Histogram: Could not load histogram from file\"" << filename << "\"." << endl;
    return false;
  }
  else {
    for (unsigned int z = 0; z < depth; ++z) {
      for (unsigned int y = 0; y < height; ++y) {
	for (unsigned int x = 0; x < width; ++x) {
	  file >> histogram[z * width * height + y * width + x];
	}
      }
    }
    file.close();
    cout << "Histogram: Loaded histogram from file \"" << filename << "\"." << endl;
    return true;
  }
}


/** Reset undo. */
void
Histogram::reset_undo()
{
  memset(undo_overlay[undo_current], 0, histogram_size);
  undo_num_vals[undo_current] = 0;
}


/** Undo. */
void
Histogram::undo()
{
  for (unsigned int z = 0; z < depth; ++z) {
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
	unsigned int index = z * width * height + y * width + x;
	histogram[index] -= undo_overlay[undo_current][index];
      }
    }
  }
  number_of_values -= undo_num_vals[undo_current];
  reset_undo();
}


/** Switch undo to another undo buffer.
 * @param undo_id switch to buffer with this ID
 * @return returns current undo buffer ID
 */
unsigned int
Histogram::switch_undo( unsigned int undo_id )
{
  unsigned int undo_last = undo_current;

  if (undo_id >= undo_num) {
    cout << "Histogram::resetUndo: ID out of range" << endl;
  } else {
    undo_current = undo_id;
  }

  return undo_last;
}


/** Get number of undos.
 * @return number of undos
 */
unsigned int
Histogram::get_num_undos()
{
  return undo_num;
}


/** Get median of all values.
 * @return median
 */
unsigned int
Histogram::get_median()
{
  vector< unsigned int > *values = new vector< unsigned int >( width * height * depth);

  values->clear();
  for (unsigned int z = 0; z < depth; ++z) {
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
	if ( histogram[z * width * height + y * width + x] > 0 ) {
	  values->push_back( histogram[z * width * height + y * width + x ] );
	}
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
Histogram::get_average()
{
  unsigned int sum = 0;
  unsigned int num = 0;
  for (unsigned int z = 0; z < depth; ++z) {
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
	if ( histogram[z * width * height + y * width + x ] ) {
	  sum += histogram[z * width * height + y * width + x ];
	  num++;
	}
      }
    }
  }
  
  return (sum / num);
}


/** Get sum of all values.
 * @return sum of values
 */
unsigned int
Histogram::get_sum() const
{
  unsigned int sum = 0;
  for (unsigned int z = 0; z < depth; ++z) {
    for (unsigned int y = 0; y < height; ++y) {
      for (unsigned int x = 0; x < width; ++x) {
	if ( histogram[z * width * height + y * width + x ] ) {
	  sum += histogram[z * width * height + y * width + x ];
	}
      }
    }
  }

  return sum;
}
