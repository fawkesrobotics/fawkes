/*
 * colorthreshold.h
 *
 *  Created on: 23.01.2014
 *      Author: Victor Mataré
 */

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

#ifndef __FIREVISION_FILTER_COLORTHRESHOLD_H_
#define __FIREVISION_FILTER_COLORTHRESHOLD_H_

#include <fvfilters/filter.h>
#include <fvutils/color/rgb.h>
#include <fvmodels/color/similarity.h>


namespace firevision
{

/**
 * A filter that uses the VLC color similarity algorithm to mark a certain color.
 * All pixels that don't match the given ColorModel are drawn in grayscale.
 */
class FilterColorThreshold : public Filter
{
  public:
    /**
     * Constructor
     * @param color_model Accepted color
     */
    FilterColorThreshold(ColorModelSimilarity *color_model);
    ~FilterColorThreshold();

    virtual void apply();
  private:
    ColorModelSimilarity *color_model_;
};

} /* namespace firevision */

#endif /* __FIREVISION_FILTER_COLORTHRESHOLD_H_ */
