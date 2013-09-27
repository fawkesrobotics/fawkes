/***************************************************************************
 *  gradient.h - Class defining a gradient (color) classifier
 *
 *  Created: Tue Jun 10 11:48:00 2008
 *  Copyright  2008 Christof Rath <christof.rath@gmail.com>
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

#include "gradient.h"
#include <core/exceptions/software.h>

using std::list;
using std::iterator;

using fawkes::upoint_t;

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class GradientClassifier <fvclassifiers/gradient.h>
 * Gradient classifier.
 * Uses the difference of the current and the last value.
 */

/** Constructor.
 * @param scanlines list of scanline models (Does only work with ScanlineGrid)
 * @param q Qualifier for a single pixel (The qualifier gets deleted by this class)
 * @param threshold minimum rise required for classification
 * @param max_size of an object to be detected (if 0 value will be ignored)
 * @param use_rising_flank
 *           if true the classification can start on a rising flank
 * @param use_falling_flank
 *           if true the classification can start on a falling flank
 */
GradientClassifier::GradientClassifier(std::list<ScanlineGrid* >* scanlines,
                                       Qualifier* q,
                                       unsigned int threshold, unsigned int max_size,
                                       bool use_rising_flank, bool use_falling_flank)
  : Classifier("GradientClassifier")
{
  if (!scanlines)
    throw fawkes::NullPointerException("GradientClassifier: scanlines may not be null!");
  if (!q)
    throw fawkes::NullPointerException("GradientClassifier: the Qualifier may not be null!");

  _scanlines = scanlines;
  _q = q;

  _max_size = 999999; //Infinite...
  set_threshold(threshold, max_size);
  set_edges(use_rising_flank, use_falling_flank);
}

/** Destructor.
 */
GradientClassifier::~GradientClassifier()
{
  if (_q)
    delete _q;
}

/** Threshold setter.
 * @param threshold minimum rise required for classification
 * @param max_size of an object to be detected (if 0 value will not be set)
 */
void
GradientClassifier::set_threshold(unsigned int threshold, unsigned int max_size)
{
  _threshold = threshold;

  if (max_size)
    _max_size = max_size;
}

/** Edge setter.
 * @param use_rising_edge
 *           if true the classification can start on a rising edge
 * @param use_falling_edge
 *           if true the classification can start on a falling edge
 */
void
GradientClassifier::set_edges(bool use_rising_edge, bool use_falling_edge)
{
  _use_rising_edge  = use_rising_edge;
  _use_falling_edge = use_falling_edge;
}

/** Set source buffer.
 * @param yuv422_planar a YUV422 planar buffer with the source image to
 * classify. The classifier may NOT modify the image in any way. If that is
 * required the classifier shall make a copy of the image.
 * @param width width of buffer in pixels
 * @param height height of buffer in pixels
 */
void
GradientClassifier::set_src_buffer(unsigned char *yuv422_planar,
                                   unsigned int width, unsigned int height)
{
  Classifier::set_src_buffer(yuv422_planar, width, height);

  _q->set_buffer(yuv422_planar, width, height);
}

std::list< ROI > *
GradientClassifier::classify()
{
  if (_q->get_buffer() == NULL)
  {
    //cout << "GradientClassifier: ERROR, src buffer not set. NOT classifying." << endl;
    return new std::list< ROI >;
  }

  list< ROI > *rv = new list< ROI >;
  int cur_val, cur_diff, direction = 0;
  upoint_t cur_pos, edge_start;
  cur_pos.x = cur_pos.y = edge_start.x = edge_start.y = 0;

  unsigned int jumpSize = 0;

  ROI current;

  for (list<ScanlineGrid*>::iterator it = _scanlines->begin(); it != _scanlines->end(); it++)
  {
    ScanlineGrid* slm = (*it);
    slm->reset();

    _last_pos = *(*slm);
    _last_val = _q->get(_last_pos);

    while(!slm->finished())
    {
      cur_pos = *(++(*slm));
      cur_val =  _q->get(cur_pos);
      cur_diff = cur_val - _last_val;

      if ((cur_pos.x < _last_pos.x || cur_pos.y < _last_pos.y) //new scan line
          || (current.pixel_step && ((cur_pos.x - current.start.x) > _max_size //area found is too big
                                     || (cur_pos.y - current.start.y) > _max_size)))
      {
        current.set_pixel_step(0);

        edge_start.x = edge_start.y = direction = jumpSize  = 0;
      }

      int curDir = (cur_diff < 0 ? -1 : (cur_diff > 0 ? 1 : 0));
      switch (curDir)
      {
      case -1:
        switch (direction)
        {
        case -1: //drop continues
          jumpSize -= cur_diff;
          break;
        case 0: //new drop
          jumpSize = -cur_diff;
          edge_start = cur_pos;
          break;
        case 1:
          if (jumpSize < _threshold)//spike reset ramp
          {
            jumpSize = -cur_diff;
            edge_start = cur_pos;
          }
          else // found edge!
          {
            if (current.pixel_step) //this is a line end
            {
              current.set_width(_last_pos.x - current.start.x);
              current.set_height(_last_pos.y - current.start.y);

              rv->push_back(ROI(current));

              current.set_pixel_step(0);
            }
            else if (_use_falling_edge)
            {
              current.set_pixel_step(1);
              current.set_start(edge_start);
            }

            edge_start = cur_pos;
            jumpSize = -cur_diff;
          }
          break;
        }
        direction = -1;
        break;


      case 0:
        switch (direction)
        {
        case -1: //ramp end
        case 1:  //ramp end
          if (jumpSize >= _threshold) //found edge!
          {
            if (current.pixel_step) //this is a line end
            {
              current.set_width(_last_pos.x - current.start.x);
              current.set_height(_last_pos.y - current.start.y);

              rv->push_back(ROI(current));

              current.set_pixel_step(0);
            }
            else
            {
              if ((_use_falling_edge && direction == 1) || (_use_rising_edge && direction == -1))
              {
                current.set_pixel_step(1);
                current.set_start(edge_start);
              }
            }
          }
          break;

        case 0:
          break;
        }
        direction = jumpSize = 0;
        edge_start.x = edge_start.y = 0;
        break;


      case 1:
        switch (direction)
        {
        case 1: //climb continues
          jumpSize += cur_diff;
          break;
        case 0: //new climb
          jumpSize = cur_diff;
          edge_start = cur_pos;
          break;
        case -1:
          if (jumpSize < _threshold)//spike reset ramp
          {
            jumpSize = cur_diff;
            edge_start = cur_pos;
          }
          else // found edge!
          {
            if (current.pixel_step) //this is a line end
            {
              current.set_width(_last_pos.x - current.start.x);
              current.set_height(_last_pos.y - current.start.y);

              rv->push_back(ROI(current));

              current.set_pixel_step(0);
            }
            else if (_use_rising_edge)
            {
              current.set_pixel_step(1);
              current.set_start(edge_start);
            }

            edge_start = cur_pos;
            jumpSize = cur_diff;
          }
          break;
        }
        direction = 1;
        break;
      }


      _last_val = cur_val;
      _last_pos = cur_pos;
    }
  } //END: For all scanline models

  return rv;
}

} // end namespace firevision
