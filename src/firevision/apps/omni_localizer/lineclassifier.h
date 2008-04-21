/*
    Copyright (c) 2008 Volker Krause <volker.krause@rwth-aachen.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*/

#ifndef FVOMNILOCALIZER_LINECLASSIFIER_H
#define FVOMNILOCALIZER_LINECLASSIFIER_H

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
