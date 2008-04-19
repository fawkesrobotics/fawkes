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

#include <apps/omni_localizer/lineclassifier.h>
#include <apps/omni_localizer/debug.h>

#include <models/scanlines/star.h>
#include <models/color/colormodel.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/color/yuv.h>
#include <fvutils/writers/jpeg.h>
#include <fvutils/draw/drawer.h>

#include <config/config.h>

#include <cassert>
#include <cmath>
#include <iostream>

using namespace std;

/** @class LineClassifier <apps/omni_localizer/lineclassifier.h>
 * Find field lines in images.
 *
 * @author Volker Krause
 */

/**
  Constructor.
  @param scanlineModel Star scanline model used for selecting the pixels to analyze.
  @param colorModel The color model used for analyzing pixels.
  @param config The configuration.
*/
LineClassifier::LineClassifier(ScanlineStar * scanlineModel, ColorModel * colorModel, Configuration *config) :
    Classifier( "LineClassifier" ),
    mLoopCount( 0 ),
    mScanlineModel( scanlineModel ),
    mColorModel( colorModel )
{
  // defaults
  config->set_default_int( "/firevision/omni/localizer/classifier/max_gaps", 2 );
  config->set_default_int( "/firevision/omni/localizer/classifier/max_line_width", 64 );

  mMaxGaps = config->get_int( "/firevision/omni/localizer/classifier/max_gaps" );
  mMaxLineWidth = config->get_int( "/firevision/omni/localizer/classifier/max_line_width" );
}

/**
  Returns the classified color at the given position.
  @param x The x coordinate.
  @param y The y coordinate.
*/
color_t LineClassifier::colorAt(unsigned x, unsigned y) const
{
  unsigned char yp = 0, up = 0, vp = 0;
  YUV422_PLANAR_YUV(_src, _width, _height, x, y, yp, up, vp);
  color_t c = mColorModel->determine(yp, up, vp);
  if ( c == C_WHITE || c == C_GREEN )
    return c;

  // ### temporary until we get C_WHITE reliably
  const float uvlow = 16.0;
  const float uvhigh = 196.0;
  const float ylow = 96.0;
  const int colr = (int)((uvlow + (((yp - ylow)/(255.0 - ylow)) * (uvhigh - uvlow))) / 2.0);
//   if ( yp > 72 && (up > 120 && up < 136) && (vp > 120 && up < 136) ) // test07
  if ( yp > ylow && (up > (128 - colr) && up < (128 + colr)) && (vp > (128 - colr) && up < (128 + colr)) )
    c = C_WHITE;
  return c;
}

/** Just for compatibility. */
std::list< ROI > * LineClassifier::classify()
{
  assert( _src );

  std::list< ROI > *rv = new std::list< ROI >;
  typedef map<float, vector<f_point_t> > HitMap;
  HitMap hits = classify2();
  for ( HitMap::const_iterator rayIt = hits.begin(); rayIt != hits.end(); ++rayIt ) {
    for ( vector<f_point_t>::const_iterator hitIt = (*rayIt).second.begin(); hitIt != (*rayIt).second.end(); ++hitIt ) {
      ROI r( (int)(*hitIt).x, (int)(*hitIt).y, 1, 1, _width, _height );
      r.set_hint( H_LINE );
      rv->push_back( r );
    }
  }

  return rv;
}

/**
  Find lines in the image.
*/
std::map< float, std::vector < f_point_t > > LineClassifier::classify2()
{
  assert( _src );

  map<float, vector<f_point_t> > rv;
  color_t c;
  bool skipAdvance = false;

#ifdef DEBUG_CLASSIFY
  unsigned char *debugBuffer;
  unsigned int debugBufferSize = colorspace_buffer_size( YUV422_PLANAR, _width, _height );
  debugBuffer = new unsigned char[debugBufferSize];
  memcpy( debugBuffer, _src, debugBufferSize * sizeof( unsigned char ) );
  Drawer debugDrawer;
  debugDrawer.setBuffer( debugBuffer, _width, _height );

#ifdef DEBUG_SEGMENTATION
  unsigned char yp = 0, up = 0, vp = 0;
  for ( unsigned int x = 0; x < _width; ++x ) {
    for ( unsigned int y = 0; y < _height; ++y ) {
      YUV422_PLANAR_YUV(_src, _width, _height, x, y, yp, up, vp);
      c = mColorModel->determine(yp,up, vp);
      switch ( c ) {
        case C_ORANGE: debugDrawer.setColor( 128, 0, 255 ); break;
        case C_BACKGROUND: debugDrawer.setColor( 128, 255, 128 ); break;
        case C_MAGENTA: debugDrawer.setColor( 128, 255, 255 ); break;
        case C_CYAN: debugDrawer.setColor( 128, 255, 0 ); break;
        case C_BLUE: debugDrawer.setColor( 128, 128, 0 ); break;
        case C_YELLOW: debugDrawer.setColor( 128, 0, 128 ); break;
        case C_GREEN: debugDrawer.setColor( 128, 0, 0 ); break;
        case C_WHITE: debugDrawer.setColor( 255, 128, 128 ); break;
        case C_RED: debugDrawer.setColor( 128, 128, 255 ); break;
        case C_BLACK: debugDrawer.setColor( 0, 128, 128 ); break;
        case C_OTHER: debugDrawer.setColor( 128, 128, 128 ); break;
      }
      debugDrawer.drawPoint( x, y );
    }
  }
#endif
#endif

  mScanlineModel->reset();
  while ( !mScanlineModel->finished() ) {

    const float currentAngle = mScanlineModel->current_angle();
    vector<f_point_t> hits;
    while ( mScanlineModel->current_angle() == currentAngle && !mScanlineModel->finished() ) {

      unsigned int x = (*mScanlineModel)->x;
      unsigned int y = (*mScanlineModel)->y;

#ifdef DEBUG_CLASSIFY
      debugDrawer.setColor( 128, 128 , 128 );
      debugDrawer.drawPoint( x, y );
#endif

      c = colorAt( x, y );
      if ( c == C_WHITE ) {
        const unsigned int startX = x;
        const unsigned int startY = y;
        unsigned int endX = startX;
        unsigned int endY = startY;

        // found a line, so let's find all following points
        int gapCount = 0;
        while ( true ) {
#ifdef DEBUG_CLASSIFY
          debugDrawer.setColor( 128, 255, 0 );
          debugDrawer.drawCircle( x, y, 1 );
#endif
          x = (*mScanlineModel)->x;
          y = (*mScanlineModel)->y;

          ++(*mScanlineModel);
          if ( mScanlineModel->finished() || mScanlineModel->current_angle() != currentAngle ) {
            skipAdvance = true;
            break;
          }

          c = colorAt( x, y );
          if ( c != C_WHITE ) {
            if ( gapCount < mMaxGaps ) {
              ++gapCount;
            } else {
              skipAdvance = true;
              break;
            }
          } else {
            endX = x;
            endY = y;
          }
        }

        // merge points into one ROI
        f_point_t hit;
        hit.x = (startX + endX) / 2.0;
        hit.y = (startY + endY) / 2.0;

        // check the size of what we found, should be at least small in one direction
        // otherwise it's probably not a line
        const int l1 = (int)sqrt((startX - endX)*(startX - endX) + (startY - endY)*(startY - endY));
        const int l2 = lineWidth( hit, currentAngle + M_PI/2.0 )
            + lineWidth( hit, currentAngle - M_PI/2.0 );
        if ( min( l1, l2 ) <= mMaxLineWidth ) {
          hits.push_back( hit );
#ifdef DEBUG_CLASSIFY
          debugDrawer.setColor( 128, 0, 255 );
          debugDrawer.drawCircle( (unsigned int)hit.x, (unsigned int)hit.y, 3 );
#endif
        }
#ifdef DEBUG_CLASSIFY
        else {
          debugDrawer.setColor( 128, 64, 128 );
          debugDrawer.drawCircle( (unsigned int)hit.x, (unsigned int)hit.y, min(l1, l2)/2 );
          // ### sin/cos are swapped here, see star.cpp
          debugDrawer.drawLine( (int)hit.x, (int)hit.y,
                                (int)(hit.x + sin( currentAngle + M_PI/2.0 ) * lineWidth( hit, currentAngle + M_PI/2.0 )),
                                (int)(hit.y + cos( currentAngle + M_PI/2.0 ) * lineWidth( hit, currentAngle + M_PI/2.0 )) );
          debugDrawer.drawLine( (int)hit.x, (int)hit.y,
                                (int)(hit.x + sin( currentAngle - M_PI/2.0 ) * lineWidth( hit, currentAngle - M_PI/2.0 )),
                                (int)(hit.y + cos( currentAngle - M_PI/2.0 ) * lineWidth( hit, currentAngle - M_PI/2.0 )) );
        }
#endif
      }

      if ( skipAdvance )
        skipAdvance = false;
      else
        ++(*mScanlineModel);
    }

    rv[currentAngle] = hits;
  }

#ifdef DEBUG_CLASSIFY
  JpegWriter writer;
  writer.set_dimensions( _width, _height );
  writer.set_buffer( YUV422_PLANAR, debugBuffer );
  char fileName[20];
  snprintf( fileName, 20, "classified-%04d.jpg", mLoopCount );
  writer.set_filename( fileName );
  writer.write();
  delete[] debugBuffer;
#endif

  return rv;
}

/**
  Determine the width of the found line at the given position.
  @param center The center of the line intersection found.
  @param angle The angle in which the line intersection was found.
*/
int LineClassifier::lineWidth(const f_point_t & center, float angle) const
{
  int rv = 0;
  int gapCount = 0;
  for ( int i = 1;; ++i ) {
    // ### sin/cos are switched here, see star.cpp
    const unsigned int x = (int)(sin( angle ) * i + center.x);
    const unsigned int y = (int)(cos( angle ) * i + center.y);
    if ( x < 0 || x > _width || y < 0 || y > _height )
      break;
    if ( colorAt( x, y ) != C_WHITE ) {
      rv = i - 1;
      if ( gapCount < mMaxGaps ) {
        ++gapCount;
      } else  {
        break;
      }
    }
  }
  return rv;
}
