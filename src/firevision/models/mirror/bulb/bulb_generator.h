
/***************************************************************************
 *  bulb_generator.h - generator for bulb lookup tables
 *
 *  Generated: Thu Mar 23 20:29:59 2006
 *  Copyright  2005-2006 Tim Niemueller [www.niemueller.de]
 *             2005      Martin Heracles
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You can read the full text in the LICENSE file in the doc directory. 
 */

#ifndef __FIREVISION_MODELS_MIRROR_BULB_BULB_GENERATOR_H_
#define __FIREVISION_MODELS_MIRROR_BULB_BULB_GENERATOR_H_

#include <fvutils/base/types.h>

#include <map>

class Bulb;
class BulbSampler;

/** Bulb generator progress handler. */
class BulbGeneratorProgressHandler {
 public:
  /** Virtual empty destructor. */
  virtual ~BulbGeneratorProgressHandler() {}
  /** Set total steps.
   * @param total_steps total number of steps
   */
  virtual void setTotalSteps(unsigned int total_steps)                  = 0;
  /** Set progress.
   * @param progress current progress
   */
  virtual void setProgress(unsigned int progress)                       = 0;
  /** Generation finished. */
  virtual void finished()                                               = 0;
};


class BulbGenerator {
 public:

  BulbGenerator(BulbSampler *sampler,
		BulbGeneratorProgressHandler *handler);
  ~BulbGenerator();

  void    generate();
  Bulb *  getResult();

 private:
  float world_distance(float dist_in_image);

  unsigned int width;
  unsigned int height;

  unsigned int center_x;
  unsigned int center_y;

  Bulb                          *data;
  Bulb                          *result;
  BulbGeneratorProgressHandler  *handler;
  BulbSampler                   *sampler;

  polar_coord_t                 *data_lut;
  polar_coord_t                 *res_lut;

  std::map<float, float> distance_table;
  float max_dist;
};



#endif
