
/***************************************************************************
 *  histogram.cpp - Implementation of the histogram
 *
 *  Generated: Tue Jun 14 11:11:29 2005
 *  Copyright  2005-2009  Tim Niemueller [www.niemueller.de]
 *             2008       Daniel Beck
 *
 ****************************************************************************/

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

#include <fvutils/statistical/histogram.h>
#include <fvutils/statistical/histogram_file.h>
#include <fvutils/statistical/histogram_block.h>

#include <core/exceptions/software.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <cstring>

using namespace std;
using namespace fawkes;


namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

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

  histogram_block = new HistogramBlock(FIREVISION_HISTOGRAM_TYPE_32, H_UNKNOWN,
               width, height, depth);
  histogram = (unsigned int*) histogram_block->data_ptr();

  histogram_size = width * height * depth * sizeof(unsigned int);

  undo_overlay = (unsigned int **)malloc(undo_num * sizeof(unsigned int *));
  for (unsigned int i = 0; i < undo_num; ++i) {
    undo_overlay[i] = (unsigned int *)malloc(histogram_size);
  }

  undo_num_vals = (unsigned int *)malloc(undo_num * sizeof(unsigned int));

  reset();
}


/** Constructor.
 * @param block construct a histogram from the given histogram block
 */
Histogram::Histogram(HistogramBlock* block)
{
  width = block->width();
  height = block->height();
  depth = block->depth();

  if (depth == 1) {
    dimension = 2;
  } else {
    dimension = 3;
  }

  undo_num = 1;
  undo_current = 0;

  histogram_block = block;
  histogram = (unsigned int*) histogram_block->data_ptr();
  histogram_size = width * height * depth * sizeof(unsigned int);

  undo_overlay = (unsigned int **)malloc(undo_num * sizeof(unsigned int *));
  for (unsigned int i = 0; i < undo_num; ++i) {
    undo_overlay[i] = (unsigned int *)malloc(histogram_size);
  }

  undo_num_vals = (unsigned int *)malloc(undo_num * sizeof(unsigned int));
}


/** Destructor. */
Histogram::~Histogram()
{
  delete histogram_block;
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
Histogram::operator+=(upoint_t *p)
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
Histogram::operator+=(upoint_t p)
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


/** Obtain the histogram block of this histogram.
 * @return pointer to the histogram block
 */
HistogramBlock *
Histogram::get_histogram_block()
{
  return histogram_block;
}


/** Obtain dimensions of the histogram.
 * @param width reference to the variable where the width is stored
 * @param height reference to the variable where the height is stored
 * @param depth reference to the variable where the depth is stored
 */
void
Histogram::get_dimensions(unsigned int& width, unsigned int& height, unsigned int& depth)
{
  width = this->width;
  height = this->height;
  depth = this->depth;
}


/** Get value from histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 * @return value
 */
unsigned int
Histogram::get_value(unsigned int x, unsigned int y)
{
  return histogram_block->get_value(x, y);
}


/** Get value from histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 * @param z z coordinate in the histogram
 * @return value
 */
unsigned int
Histogram::get_value(unsigned int x, unsigned int y, unsigned int z)
{
  return histogram_block->get_value(x, y, z);
}


/** Set value in histogram.
 * @param x x coordinate in histogram plane
 * @param y y coordinate in histogram plane
 * @param value value
 */
void
Histogram::set_value(unsigned int x, unsigned int y, unsigned int value)
{
  unsigned int old_value = histogram_block->get_value(x, y);
  histogram_block->set_value(x, y, value);
  number_of_values += value - old_value;

  unsigned int index = y * width + x;
  if ( value > old_value ) {
    // The value got incremented, add to overlay
    undo_overlay[undo_current][index] += value - old_value;
  } else {
    if ( (old_value - value) < undo_overlay[undo_current][index] ) {
      undo_overlay[undo_current][index] -= (old_value - value);
    } else {
      // We cannot handle values smaller than the original value, this is
      // due to choosing unsigned int as datatype, right now this should suffice
      undo_overlay[undo_current][index] = 0;
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
  unsigned int old_value = histogram_block->get_value(x, y, z);
  histogram_block->set_value(x, y, z, value);

  number_of_values += value - old_value;
  unsigned int index = z * width * height + y * width + x;
  if ( value > old_value ) {
    // The value got incremented, add to overlay
    undo_overlay[undo_current][index] += value - old_value;
  } else {
    if ( (old_value - value) < undo_overlay[undo_current][index] ) {
      undo_overlay[undo_current][index] -= (old_value - value);
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
  unsigned int old_value = histogram_block->get_value(x, y, z);
  histogram_block->set_value(x, y, z, ++old_value);

  ++number_of_values;

  unsigned int index = z * width * height + y * width + x;
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
  unsigned int cur_value = histogram_block->get_value(x, y, z);
  histogram_block->set_value(x, y, z, cur_value + value);

  number_of_values += value;

  unsigned int index = z * width * height + y * width + x;
  undo_overlay[undo_current][index] = value;
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
  unsigned int cur_value = histogram_block->get_value(x, y, z);
  if (value < cur_value) {
    set_value(x, y, z, cur_value - value);
  } else {
    set_value(x, y, z, 0);
  }

  number_of_values -= value;

  unsigned int index = z * width * height + y * width + x;
  if ( value < undo_overlay[undo_current][index] )
    {
      undo_overlay[undo_current][index] -= value;
    }
  else
    {
      undo_overlay[undo_current][index] = 0;
    }
}


/** Reset histogram. */
void
Histogram::reset()
{
  histogram_block->reset();
  //  memset(histogram, 0, histogram_size);
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
//  cout << histogram[z * width * height + y * width + x] << " ";
  cout << histogram_block->get_value(x, y, z) << " ";
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
Histogram::save(const char *filename, bool formatted_output)
{
  HistogramFile histogram_file;
  histogram_file.set_owns_blocks(false);
  histogram_file.add_histogram_block(histogram_block);
  histogram_file.write(filename);

  cout << "Histogram: Saved histogram in file \"" << filename << "\"." << endl;
}


/** Load from file.
 * @param filename file name to read from
 * @return true on success, false otherwise
 */
bool
Histogram::load(const char *filename)
{
  HistogramFile histogram_file;
  histogram_file.read(filename);

  if ( histogram_file.num_blocks() != 1 )
    {
      printf("load() aborted: file contains more than one histogram");
      return false;
    }

  histogram_block = (HistogramBlock*) histogram_file.blocks().front();
  histogram = (unsigned int*) histogram_block->data_ptr();
  histogram_size = width * height * depth * sizeof(unsigned int);

  for (unsigned int i = 0; i < undo_num; ++i) {
    free(undo_overlay[i]);
  }
  free(undo_overlay);

  undo_overlay = (unsigned int **)malloc(undo_num * sizeof(unsigned int *));
  for (unsigned int i = 0; i < undo_num; ++i) {
    undo_overlay[i] = (unsigned int *)malloc(histogram_size);
  }

  return true;
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
  values->push_back( histogram_block->get_value(x, y, z) );
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
    sum += histogram_block->get_value(x, y, z);
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
    sum += histogram_block->get_value(x, y, z);
  }
      }
    }
  }

  return sum;
}

} // end namespace firevision
