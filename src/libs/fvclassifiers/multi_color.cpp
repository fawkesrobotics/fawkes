
/***************************************************************************
 *  multi_color.cpp - Implementation of a MultiColorClassifier
 *
 *  Created: Sat Apr 02 09:57:14 2011
 *  Copyright  2005-2011  Tim Niemueller [www.niemueller.de]
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

#include <fvclassifiers/multi_color.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/color/yuv.h>
#include <fvutils/color/color_object_map.h>

#include <fvmodels/scanlines/scanlinemodel.h>
#include <fvmodels/color/colormodel.h>

#include <core/exceptions/software.h>

#include <cstdlib>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class MultiColorClassifier <fvclassifiers/multi_color.h>
 * Simple multi-color classifier.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param scanline_model scanline model
 * @param color_model color model
 * @param min_num_points minimum number of points in ROI to be considered
 * @param box_extent basic extent of a new ROI
 * @param upward set to true if you have an upward scanline model, this means that
 * points are traversed from the bottom to the top. In this case the ROIs are initially
 * extended towards the top instead of the bottom.
 * @param neighbourhood_min_match minimum number of object pixels to grow neighbourhood
 * @param grow_by grow region by that many pixels
 */
MultiColorClassifier::MultiColorClassifier(ScanlineModel *scanline_model,
             ColorModel *color_model,
             unsigned int min_num_points,
             unsigned int box_extent,
             bool upward,
             unsigned int neighbourhood_min_match,
             unsigned int grow_by)
  : Classifier("MultiColorClassifier")
{
  if (scanline_model == NULL) {
    throw fawkes::NullPointerException("MultiColorClassifier: scanline_model "
               "may not be NULL");
  }
  if (color_model == NULL) {
    throw fawkes::NullPointerException("MultiColorClassifier: color_model "
               "may not be NULL");
  }

  modified = false;
  this->scanline_model = scanline_model;
  this->color_model = color_model;
  this->min_num_points = min_num_points;
  this->box_extent = box_extent;
  this->upward = upward;
  this->grow_by = grow_by;
  this->neighbourhood_min_match = neighbourhood_min_match;
}


unsigned int
MultiColorClassifier::consider_neighbourhood( unsigned int x, unsigned int y , color_t what)
{
  color_t c;
  unsigned int num_what = 0;

  unsigned char yp = 0, up = 0, vp = 0;
  int start_x = -2, start_y = -2;
  int end_x = 2, end_y = 2;

  if (x < (unsigned int)abs(start_x)) {
    start_x = 0;
  }
  if (y < (unsigned int)abs(start_y)) {
    start_y = 0;
  }

  if (x > _width - end_x) {
    end_x = 0;
  }
  if (y == _height - end_y) {
    end_y = 0;
  }

  for (int dx = start_x; dx <= end_x ; dx += 2) {
    for (int dy = start_y; dy <= end_y; ++dy) {
      if ((dx == 0) && (dy == 0)) {
        continue;
      }

      //      cout << "x=" << x << "  dx=" << dx << "  +=" << x+dx
      //   << "  y=" << y << "  dy=" << dy << "  +2=" << y+dy << endl;

      YUV422_PLANAR_YUV(_src, _width, _height, x+dx, y+dy, yp, up, vp);
      c = color_model->determine(yp, up, vp);

      if (c == what) {
        ++num_what;
      }
    }
  }

  return num_what;
}

std::list< ROI > *
MultiColorClassifier::classify()
{

  if (_src == NULL) {
    return new std::list< ROI >;
  }

  std::map<color_t, std::list<ROI> > rois;
  std::map<color_t, std::list<ROI> >::iterator map_it;
  std::list<ROI>::iterator roi_it, roi_it2;
  color_t c;

  unsigned int  x = 0, y = 0;
  unsigned char yp = 0, up = 0, vp = 0;
  unsigned int num_what = 0;

  ROI r;

  scanline_model->reset();
  while (! scanline_model->finished()) {

    x = (*scanline_model)->x;
    y = (*scanline_model)->y;

    YUV422_PLANAR_YUV(_src, _width, _height, x, y, yp, up, vp);

    c = color_model->determine(yp,up, vp);

    if ((c != C_BACKGROUND) && (c != C_OTHER)) {
      // Yeah, found something, make it big and name it a ROI
      // Note that this may throw out a couple of ROIs for just one ball,
      // as the name suggests this one is really ABSOLUTELY simple and not
      // useful for anything else than quick testing

      if (neighbourhood_min_match) {
        num_what =
    consider_neighbourhood((*scanline_model)->x, (*scanline_model)->y, c);
      }
      if (num_what >= neighbourhood_min_match) {
        bool ok = false;

        for (roi_it = rois[c].begin(); roi_it != rois[c].end(); ++roi_it) {
          if ( (*roi_it).contains(x, y) ) {
            // everything is fine, this point is already in another ROI
            ok = true;
            break;
          }
        }
        if (! ok) {
          for (roi_it = rois[c].begin(); roi_it != rois[c].end(); ++roi_it) {
            if ( (*roi_it).neighbours(x, y, scanline_model->get_margin()) ) {
              // ROI is neighbour of this point, extend region
              (*roi_it).extend(x, y);
              ok = true;
              break;
            }
          }
        }

        if (! ok) {
          if ( upward ) {
            if ( x < box_extent ) {
              x = 0;
            } else {
              x -= box_extent;
            }
            if ( y < box_extent ) {
              y = 0;
            } else {
              y -= box_extent;
            }
          }
          r.start.x = x;
          r.start.y = y;

          unsigned int to_x = (*scanline_model)->x + box_extent;
          unsigned int to_y = (*scanline_model)->y + box_extent;
          if (to_x > _width)  to_x = _width;
          if (to_y > _height) to_y = _height;
          r.width = to_x - r.start.x;
          r.height = to_y - r.start.y;
          r.hint = c;
    r.color = c;

          r.line_step = _width;
          r.pixel_step = 1;

          r.image_width  = _width;
          r.image_height = _height;

          if ( (r.start.x + r.width) > _width ) {
            r.width -= (r.start.x + r.width) - _width;
          }
          if ( (r.start.y + r.height) > _height ) {
            r.height -= (r.start.y + r.height) - _height;
          }

          rois[c].push_back( r );
        }
      } // End if enough neighbours
    } // end if is orange

    ++(*scanline_model);
  }

  // Grow regions
  if (grow_by > 0) {
    for (map_it = rois.begin(); map_it != rois.end(); ++map_it) {
      for (roi_it = map_it->second.begin();
     roi_it != map_it->second.end(); ++roi_it)
      {
  (*roi_it).grow( grow_by );
      }
    }
  }

  // Merge neighbouring regions
  for (map_it = rois.begin(); map_it != rois.end(); ++map_it) {
    for (roi_it = map_it->second.begin(); roi_it != map_it->second.end(); ++roi_it)
    {
      roi_it2 = roi_it;
      ++roi_it2;

      while ( roi_it2 != map_it->second.end() ) {
  if ((roi_it != roi_it2) &&
      roi_it->neighbours(&(*roi_it2), scanline_model->get_margin()))
  {
    *roi_it += *roi_it2;
    map_it->second.erase(roi_it2);
    roi_it2 = map_it->second.begin(); //restart
  } else {
    ++roi_it2;
  }
      }
    }
  }

  // Throw away all ROIs that have not enough classified points
  for (map_it = rois.begin(); map_it != rois.end(); ++map_it) {
    for (roi_it = map_it->second.begin(); roi_it != map_it->second.end(); ++roi_it)
    {
      while ( (roi_it != map_it->second.end()) &&
        ((*roi_it).num_hint_points < min_num_points ))
      {
  roi_it = map_it->second.erase(roi_it);
      }
    }
  }

  // sort ROIs by number of hint points, descending (and thus call reverse)

  std::list<ROI> *rv = new std::list<ROI>();
  for (map_it = rois.begin(); map_it != rois.end(); ++map_it) {
    map_it->second.sort();
    rv->merge(map_it->second);
  }
  rv->reverse();
  return rv;
}


/** Get mass point of primary color.
 * @param roi ROI to consider
 * @param massPoint contains mass point upon return
 */
void
MultiColorClassifier::get_mass_point_of_color( ROI *roi,
                 fawkes::upoint_t *massPoint )
{
  unsigned int nrOfOrangePixels;
  nrOfOrangePixels = 0;
  massPoint->x     = 0;
  massPoint->y     = 0;

  // for accessing ROI pixels
  unsigned int h = 0;
  unsigned int w = 0;
  // planes
  unsigned char *yp   = _src + (roi->start.y * roi->line_step) + (roi->start.x * roi->pixel_step);
  unsigned char *up   = YUV422_PLANAR_U_PLANE(_src, roi->image_width, roi->image_height)
    + ((roi->start.y * roi->line_step) / 2 + (roi->start.x * roi->pixel_step) / 2) ;
  unsigned char *vp   = YUV422_PLANAR_V_PLANE(_src, roi->image_width, roi->image_height)
    + ((roi->start.y * roi->line_step) / 2 + (roi->start.x * roi->pixel_step) / 2);
  // line starts
  unsigned char *lyp  = yp;
  unsigned char *lup  = up;
  unsigned char *lvp  = vp;

  color_t dcolor;

  // consider each ROI pixel
  for (h = 0; h < roi->height; ++h) {
    for (w = 0; w < roi->width; w += 2) {
      // classify its color
      dcolor = color_model->determine(*yp++, *up++, *vp++);
      yp++;
      // ball pixel?
      if (dcolor == roi->color) {
        // take into account its coordinates
        massPoint->x += w;
        massPoint->y += h;
        nrOfOrangePixels++;
      }
    }
    // next line
    lyp  += roi->line_step;
    lup  += roi->line_step / 2;
    lvp  += roi->line_step / 2;
    yp    = lyp;
    up    = lup;
    vp    = lvp;
  }

  // to obtain mass point, divide by number of pixels that were added up
  massPoint->x = (unsigned int) (float(massPoint->x) / float(nrOfOrangePixels));
  massPoint->y = (unsigned int) (float(massPoint->y) / float(nrOfOrangePixels));

  /* shift mass point
   such that it is relative to image
   (not relative to ROI) */
  massPoint->x += roi->start.x;
  massPoint->y += roi->start.y;
}

} // end namespace firevision
