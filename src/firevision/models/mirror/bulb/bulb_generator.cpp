
/***************************************************************************
 *  bulb_generator.cpp - generator for bulb lookup tables
 *
 *  Generated: Thu Mar 23 20:40:27 2006
 *  Copyright  2005-2006 Tim Niemueller [www.niemueller.de]
 *             2005      Martin Heracles
 *             2008      Daniel Beck
 *
 *  $Id$
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

#include <models/mirror/bulb/bulb_generator.h>
#include <models/mirror/bulb/bulb_sampler.h>
#include <models/mirror/bulb.h>

#include <utils/math/angle.h>

#include <cstdlib>
#include <cmath>
#include <list>
#include <utility>

using namespace std;
using namespace fawkes;

/** @class BulbGenerator <models/mirror/bulb/bulb_generator.h>
 * Bulb Lookuptable Generator.
 * This generator processes samples taken with BulbSampler and calculates the
 * mirror lookup table.
 * @author Martin Herakles
 * @author Tim Niemueller
 * @author Daniel Beck
 */

/** Constructor.
 * @param sampler bulb sampler
 * @param handler progress handler, will informed about progress
 */
BulbGenerator::BulbGenerator(BulbSampler *sampler,
			     BulbGeneratorProgressHandler *handler)
{
  this->sampler = sampler;
  this->handler = handler;

  data   = sampler->getBulb();
  result = new Bulb(*data);

  width    = data->width;
  height   = data->height;
  center_x = data->image_center_x;
  center_y = data->image_center_y;

  data_lut = data->lut;
  res_lut = result->lut;
}


/** Destructor. */
BulbGenerator::~BulbGenerator()
{
  delete result;
}


/** Generate LUT. */
void
BulbGenerator::generate()
{

  unsigned int totalNrOfPixels = width * height;
  unsigned int nrOfProcessedPixels = 0;

  handler->setTotalSteps(totalNrOfPixels);
  
  max_dist = 0.0f;
  distance_table.clear();

  // enter trained distances in distance table
  printf("entering trained distances in distance table\n");
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      if ( data->isNonZero(x, y) ) {
	float image_dist = (float) rint( data->getDistanceInImage(x, y, center_x, center_y) * 10 ) / 10.0f;
	if (image_dist > max_dist)
	  { max_dist = image_dist; }	    
	polar_coord_t p = data->getWorldPointRelative(x, y);
	distance_table[image_dist] = p.r;
      }
    }
  }
  distance_table[0] = 0.01f;

  // interpolate all other
  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      float dist = world_distance( data->getDistanceInImage(x, y, center_x, center_y) );
      if (dist < 0)
	{ continue; }
      float phi = atan2f( float(x) - float(center_x), float(center_y) - float(y) );				   
      result->setWorldPoint(x, y, dist, phi);

      // whether interpolated or not, count this pixel as processed
      ++nrOfProcessedPixels;
    }
    handler->setProgress(nrOfProcessedPixels);
  }
  handler->finished();
}


/** Get the result.
 * @return Bulb mirror model.
 */
Bulb *
BulbGenerator::getResult()
{
  return result;
}


float
BulbGenerator::world_distance(float dist_in_image)
{
  if ( distance_table.size() == 0)
    {
      printf("No values in distance table\n");
      return -1.0f;
    }

  float image_dist = (float) rint(dist_in_image * 10) / 10.0f;

  if (image_dist > max_dist)
    { return -1.0f; }

  if ( distance_table.find(image_dist) != distance_table.end() ) 
    { return distance_table[image_dist]; }

  std::map<float, float>::iterator lower, upper;

  lower = distance_table.lower_bound(image_dist);
  if ( lower != distance_table.begin() )
    { --lower; }
  else
    { printf("lower not found\n"); }

  upper = distance_table.upper_bound(image_dist);

  float world_dist;
  if (lower != distance_table.end() &&
      upper != distance_table.end() )
    {
      float factor;
      factor = (image_dist - lower->first) / (float)(upper->first - lower->first);
      world_dist =  factor * upper->second + ( 1.0 - factor ) * lower->second;
      
      distance_table[image_dist] = world_dist;
    }
  else
    {
      printf("upper/lower not found\n");
    }

  return world_dist;
}
