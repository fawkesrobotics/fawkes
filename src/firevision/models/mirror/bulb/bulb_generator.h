
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

#ifndef __FIREVISION_MODELS_MIRROR_BULB_BULB_GENERATOR_H_
#define __FIREVISION_MODELS_MIRROR_BULB_BULB_GENERATOR_H_

#include <fvutils/base/types.h>

class Bulb;
class BulbSampler;

/** Bulb generator progress handler. */
class BulbGeneratorProgressHandler {
 public:
  /** Virtual empty destructor. */
  virtual ~BulbGeneratorProgressHandler() {}
  /** Set total steps */
  virtual void setTotalSteps(unsigned int total_steps)                  = 0;
  /** Set progress */
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

};



#endif
