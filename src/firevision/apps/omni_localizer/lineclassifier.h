
/***************************************************************************
 *  lineclassifier.h - line classifier
 *
 *  Created: ???
 *  Copyright  2008  Volker Krause <volker.krause@rwth-aachen.de>
 *
 *  $Id: pipeline_thread.cpp 1049 2008-04-24 22:40:12Z beck $
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

#ifndef __FIREVISION_APPS_OMNI_LOCALIZER_LINECLASSIFIER_H_
#define __FIREVISION_APPS_OMNI_LOCALIZER_LINECLASSIFIER_H_

#include <classifiers/classifier.h>
#include <fvutils/base/types.h>

#include <map>
#include <vector>

class Configuration;
class ScanlineStar;
class ColorModel;

class LineClassifier : public Classifier
{
  public:
    LineClassifier( ScanlineStar *scanlineModel,
                    ColorModel *colorModel,
                    Configuration *config );

    std::list< ROI > * classify();

    std::map<float, std::vector<f_point_t> > classify2();

    /** Debug hack, remove at some point. */
    int mLoopCount; // ### DEBUG HACK

  private:
    color_t colorAt( unsigned x, unsigned y ) const;
    int lineWidth( const f_point_t &center, float angle ) const;

  private:
    ScanlineStar *mScanlineModel;
    ColorModel   *mColorModel;
    int mMaxGaps;
    int mMaxLineWidth;

    float mUVLow, mUVHigh;
    float mYLow;
};


#endif
