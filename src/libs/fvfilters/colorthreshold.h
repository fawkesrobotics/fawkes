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

class FilterColorThreshold : public Filter
{
  public:
    FilterColorThreshold(RGB_t reference_color, int chroma_threshold, int saturation_threshold);
    ~FilterColorThreshold();

    virtual void apply();

    void set_similarity_threshold(int thres);
    void set_saturation_threshold(int thres);
    void set_reference_color(RGB_t color);
  private:
    int chroma_thresh_;
    int saturation_thresh_;
    int ref_u_;
    int ref_v_;
    int ref_len_;
};

} /* namespace firevision */

#endif /* __FIREVISION_FILTER_COLORTHRESHOLD_H_ */
