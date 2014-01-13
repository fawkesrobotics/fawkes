
/***************************************************************************
 *  shrinker.cpp - Implementation of Shrinker
 *
 *  Generated: Wed Aug 31 2005 21:52:28
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include <fvclassifiers/shrinker.h>
#include <fvutils/color/colorspaces.h>
#include <fvutils/base/roi.h>

#include <cstddef>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class Shrinker <fvclassifiers/shrinker.h>
 * Shrinker class to shrink ROIs.
 * This shrinker shrinks a given ROI. This is done to cope with several
 * special problems that arise in different setups. For example if playing
 * downstairs in the lobby without a carpet we always have a problem with
 * reflections on the floor.
 *
 * This shrinker works like this:
 * - if ROI is vertically rectangular, we cut off the bottom part
 *   because it is likely to contain reflection
 * - if ball is not close (roi->width <= 100), we tighten the ROI, such that
 *   it only contains the ball.  This helps against reflection.
 *   (If ball is close, this does not work, because it takes away too many edge pixels.)
 */

/** Constructor. */
Shrinker::Shrinker()
{
  src = NULL;
}


/** Destructor. */
Shrinker::~Shrinker()
{
}


/** Set the filtered buffer.
 * The buffer is assumed to being YUV422_PLANAR mode and the desired filter
 * combination has been run.
 * @param yuv422planar_buffer YUV422 planar buffer
 */
void
Shrinker::setFilteredBuffer(unsigned char *yuv422planar_buffer) {
  src = yuv422planar_buffer;
}


/** Shrink!
 * Do the actual shrinking. See above for used method.
 * @param roi ROI to srhink
 */
void
Shrinker::shrink( ROI *roi )
{

  unsigned int x;
  unsigned int y;

  /* if ROI is vertically rectangular, we cut off the bottom part
     because it is likely to contain reflection */
  if (roi->height > roi->width) {
    roi->height = roi->width;
  }

  if ( roi->width <= 100 ) {
    /* Ball is not close. Tighten ROI, such that it only contains the ball.
       This helps against reflection.
       (If ball is close, this does not work, because it takes away too many edge pixels.) */

    unsigned char  *bufferTmp     = roi->get_roi_buffer_start( src );
    unsigned char  *line_startTmp = bufferTmp;
    fawkes::upoint_t leftmostPixel = {roi->width, 0};
    fawkes::upoint_t topmostPixel  = {0, roi->height};

    // find leftmost hint-pixel
    bool pixelFound = false;
    for (x = 0; !pixelFound && (x < roi->width / 2); ++x) {
      for (y = 0; y < roi->height/2; ++y) {
        if (*bufferTmp > 230) { // if pixel is white or almost white = edge
          leftmostPixel.x = x;
          leftmostPixel.y = y;
          pixelFound = true;
        }
        ++bufferTmp;
      } // inner for
      line_startTmp += roi->line_step;
      bufferTmp = line_startTmp;
    } // outer for

    bufferTmp = roi->get_roi_buffer_start( src );
    line_startTmp = bufferTmp;

    // find topmost hint-pixel
    pixelFound = false;
    for (y = 0; !pixelFound && (y < roi->height/2); ++y) {
      for (x = 0; x < roi->width; ++x) {
        if (*bufferTmp > 230) {
          topmostPixel.x = x;
          topmostPixel.y = y;
          pixelFound = true;
        }
        ++bufferTmp;
      } // inner for
      /*
      if (pixelFound) {
        // try to improve x-coordinate (too small)
        unsigned int x2 = topmostPixel.x;
        for (unsigned int a = topmostPixel.x + 1; a < roi->width; ++a) {
        if (TEST_IF_IS_A_PIXEL(*bufferTmp)) {
        x2 = a;
        }
        ++bufferTmp;
        }
        topmostPixel.x = (topmostPixel.x + x2) / 2;
      }
      */
      line_startTmp += roi->line_step;
      bufferTmp = line_startTmp;
    } // outer for

    bufferTmp = roi->get_roi_buffer_start( src );
    line_startTmp = bufferTmp;

    // tighten ROI if it makes sense
    if ( (leftmostPixel.x >= topmostPixel.x)  ||
         (topmostPixel.y  >= leftmostPixel.y) ||
         (2 * (topmostPixel.x - leftmostPixel.x) >= roi->width) ||
         (2 * (leftmostPixel.y - topmostPixel.y) >= roi->height) ) {
      // bad pixels found
    } else {
      // tighten ROI
      // roi->start.x += leftmostPixel.x;
      // roi->start.y += topmostPixel.y;
      // roi->height -= topmostPixel.y;
      //roi->width = 2 * (topmostPixel.x - leftmostPixel.x);
      roi->height = 2 * (leftmostPixel.y - topmostPixel.y);
      /*
        if ( roi->width < roi->height ) {
        // further shrinking
        roi->height = roi->width;
        }
      */
      //cout << "Shrinker: Shrank region!" << endl;
    }
  } // else do nothing

}

} // end namespace firevision
