
/***************************************************************************
 *  filter.cpp - Laser data filter interface
 *
 *  Created: Fri Oct 10 17:12:29 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "filter.h"
#include <core/exception.h>

#include <cstdlib>

/** @class LaserDataFilter "filter.h"
 * Laser data filter.
 * With this interface laser filter are described. These filters take laser
 * readings as input, mangle them and return a new array of filtered laser data.
 * @author Tim Niemueller
 *
 * @fn void LaserDataFilter::filter() = 0
 * Filter the incoming data.
 * Function shall filter the data in the "in" member vector and write output
 * to the "out" member vector.
 */

/** @var LaserDataFilter::in
 * Vector of input arrays.
 * Each entry in the vector is an array of data_size entries. It depends on
 * the filter how multiple inputs are processed.
 */

/** @var LaserDataFilter::out
 * Vector of output arrays.
 * Each entry in the vector is an array of data_size entries. It depends on
 * the filter how multiple outputs are generated.
 */

/** @var LaserDataFilter::in_data_size
 * Number of entries in input arrays.
 */

/** @var LaserDataFilter::out_data_size
 * Number of entries in output arrays.
 */

/** Constructor.
 * @param in_data_size number of entries input value arrays
 * @param in vector of input arrays
 * @param out_size number of value arrays to generate in out vector
 */
LaserDataFilter::LaserDataFilter(unsigned int in_data_size,
				 std::vector<float *> in, unsigned int out_size)
{
  this->in            = in;
  this->in_data_size  = in_data_size;
  this->out_data_size = in_data_size; // yes, in_data_size!

  if (out_size > 0)  out.resize(out_size);
  for (unsigned int i = 0; i < out_size; ++i) {
    out[i] = (float *)malloc(out_data_size * sizeof(float));
  }

  __own_in  = false;
  __own_out = true;
}


/** Virtual empty destructor. */
LaserDataFilter::~LaserDataFilter()
{
  if (__own_in) {
    for (unsigned int i = 0; i < in.size(); ++i) {
      free(in[i]);
    }
  }
  if (__own_out) {
    for (unsigned int i = 0; i < out.size(); ++i) {
      free(out[i]);
    }
  }
}


/** Get filtered data array
 * @return a float array of the same size as the last array given to filter()
 * or NULL if filter() was never called.
 */
std::vector<float *> &
LaserDataFilter::get_out_vector()
{
  return out;
}

/** Set filtered data array
 * @param out vector of output values. The vector is only accepted if it has
 * the same size as the current one. The filter will now longer assume
 * ownership of the arrays in the vector. Either free the memory or call
 * set_array_ownership().
 */
void
LaserDataFilter::set_out_vector(std::vector<float *> &out)
{
  if (this->out.size() != out.size()) {
    throw fawkes::Exception("Filter out vector size mismatch: %zu vs. %zu",
			    this->out.size(), out.size());
  }

  if (__own_out) {
    for (unsigned int i = 0; i < this->out.size(); ++i) {
      free(this->out[i]);
    }
  }
  this->out.clear();

  this->out = out;
  __own_out = false;
}


/** Resize output arrays.
 * A side effect is that the output array size will be owned afterwards.
 * Call this method only in constructors! Note that the output arrays are
 * only recreated if own by the filter. If you passed an out vector you have
 * to make sure the contained arrays fit (before calling set_out_vector()!).
 * @param data_size number of entries in output arrays.
 */
void
LaserDataFilter::set_out_data_size(unsigned int data_size)
{
  if (out_data_size != data_size) {
    if (__own_out) {
      for (unsigned int i = 0; i < out.size(); ++i) {
	free(out[i]);
	out[i] = (float *)malloc(data_size * sizeof(float));
      }
    }
  }

  out_data_size = data_size;
}


/** Get size of filtered data array
 * @return size of filtered data array or 0 if filter() was never called.
 */
unsigned int
LaserDataFilter::get_out_data_size()
{
  return out_data_size;
}


/** Set input/output array ownership.
 * Owned arrays will be freed on destruction or when setting new arrays.
 * @param own_in true to assign ownership of input arrays, false otherwise
 * @param own_out true to assign ownership of output arrays, false otherwise
 */
void
LaserDataFilter::set_array_ownership(bool own_in, bool own_out)
{
  __own_in  = own_in;
  __own_out = own_out;
}
