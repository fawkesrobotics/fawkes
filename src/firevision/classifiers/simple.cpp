
/***************************************************************************
 *  simple.cpp - Implementation of a ReallySimpleClassifier
 *
 *  Generated: Thu May 16 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
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

#include <classifiers/simple.h>

#include <fvutils/color/colorspaces.h>
#include <fvutils/color/yuv.h>

#include <models/scanlines/scanlinemodel.h>
#include <models/color/colormodel.h>

#include <iostream>
using namespace std;

/** @class ReallySimpleClassifier <classifiers/simple.h>
 * Simple classifier.
 */

/** Constructor.
 * @param width image width
 * @param height image height
 * @param scanline_model scanline model
 * @param color_model color model
 * @param min_num_points minimum number of points in ROI to be considered
 * @param box_extent basic extent of a new ROI
 * @param neighbourhood_min_match minimum number of object pixels to grow neighbourhood
 * @param grow_by grow region by that many pixels
 */
ReallySimpleClassifier::ReallySimpleClassifier(unsigned int width,
					       unsigned int height,
					       ScanlineModel *scanline_model,
					       ColorModel *color_model,
					       unsigned int min_num_points,
					       unsigned int box_extent,
					       bool upward,
					       unsigned int neighbourhood_min_match,
                                               unsigned int grow_by
					       )
{
  src = NULL;
  modified = false;
  this->scanline_model = scanline_model;
  this->color_model = color_model;
  this->width = width;
  this->height = height;
  this->min_num_points = min_num_points;
  this->box_extent = box_extent;
  this->upward = upward;
  this->grow_by = grow_by;
  this->neighbourhood_min_match = neighbourhood_min_match;
}


void
ReallySimpleClassifier::setSrcBuffer(unsigned char *buf) {
  src = buf;
}


const char *
ReallySimpleClassifier::getName() const
{
  return "ReallySimpleClassifier";
}


unsigned int
ReallySimpleClassifier::consider_neighbourhood( unsigned int x, unsigned int y , color_t what)
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

  if (x > width - end_x) {
    end_x = 0;
  }
  if (y == height - end_y) {
    end_y = 0;
  }

  for (int dx = start_x; dx <= end_x ; dx += 2) {
    for (int dy = start_y; dy <= end_y; ++dy) {
      if ((dx == 0) && (dy == 0)) {
	continue;
      }

      //      cout << "x=" << x << "  dx=" << dx << "  +=" << x+dx
      //   << "  y=" << y << "  dy=" << dy << "  +2=" << y+dy << endl;

      YUV422_PLANAR_YUV(src, width, height, x+dx, y+dy, yp, up, vp);
      c = color_model->determine(yp, up, vp);

      if (c == what) {
	++num_what;
      }
    }
  }

  return num_what;
}

std::list< ROI > *
ReallySimpleClassifier::classify()
{

  if (src == NULL) {
    //cout << "ReallySimpleClassifier: ERROR, src buffer not set. NOT classifying." << endl;
    return new std::list< ROI >;
  }


  std::list< ROI > *rv = new std::list< ROI >;
  std::list< ROI >::iterator roi_it, roi_it2;
  color_t c;
  rv->clear();

  unsigned int  x = 0, y = 0;
  unsigned char yp = 0, up = 0, vp = 0;
  unsigned int num_what;

  ROI r;

  scanline_model->reset();
  while (! scanline_model->finished()) {

    x = (*scanline_model)->x;
    y = (*scanline_model)->y;

    YUV422_PLANAR_YUV(src, width, height, x, y, yp, up, vp);

    /*
    cout << "ReallySimpleClassifier: Checking at ("
	 << x
	 << ","
	 << y
	 << ") "
	 << " with color Y=" << (int)macro_pixel[y_pos]
	 << "  U=" << (int)macro_pixel[0]
	 << "  V=" << (int)macro_pixel[2]
	 << endl;
    */    

    c = color_model->determine(yp,up, vp);

    if (c == C_ORANGE) {
      // Yeah, found a ball, make it big and name it a ROI
      // Note that this may throw out a couple of ROIs for just one ball,
      // as the name suggests this one is really ABSOLUTELY simple and not
      // useful for anything else than quick testing

      num_what = consider_neighbourhood((*scanline_model)->x, (*scanline_model)->y, C_ORANGE);
      if (num_what > neighbourhood_min_match) {

	bool ok = false;

	for (roi_it = rv->begin(); roi_it != rv->end(); ++roi_it) {
	  if ( (*roi_it).contains(x, y) ) {
	    // everything is fine, this point is already in another ROI
	    ok = true;
	    break;
	  }
	}
	if (! ok) {
	  for (roi_it = rv->begin(); roi_it != rv->end(); ++roi_it) {
	    if ( (*roi_it).neighbours(x, y, scanline_model->getMargin()) ) {
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
	  if (to_x > width)  to_x = width;
	  if (to_y > height) to_y = height;
	  r.width = to_x - r.start.x;
	  r.height = to_y - r.start.y;
	  r.hint = H_BALL;

	  r.line_step = width;
	  r.pixel_step = 1;

	  r.image_width  = width;
	  r.image_height = height;

	  if ( (r.start.x + r.width) > width ) {
	    r.width -= (r.start.x + r.width) - width;
	  }
	  if ( (r.start.y + r.height) > height ) {
	    r.height -= (r.start.y + r.height) - height;
	  }

	  rv->push_back( r );
	}
      } // End if enough neighbours
    } // end if is orange

    ++(*scanline_model);
  }

  // Grow regions
  if (grow_by > 0) {
    for (roi_it = rv->begin(); roi_it != rv->end(); ++roi_it) {
      (*roi_it).grow( grow_by );
    }
  }
  
  // Merge neighbouring regions
  for (roi_it = rv->begin(); roi_it != rv->end(); ++roi_it) {
    roi_it2 = roi_it;
    ++roi_it2;

    while ( roi_it2 != rv->end() ) {
      if (roi_it->neighbours(&(*roi_it2), scanline_model->getMargin())) {
	*roi_it += *roi_it2;
	roi_it2 = rv->erase(roi_it2);
      } else {
	++roi_it2;
      }
    }
  }
  
  // Throw away all ROIs that have not enough classified points
  for (roi_it = rv->begin(); roi_it != rv->end(); ++roi_it) {
    while ( (roi_it != rv->end()) &&
	    ((*roi_it).num_hint_points < min_num_points )) {
      roi_it = rv->erase( roi_it );
    }
  }

  // sort ROIs by number of hint points, descending (and thus call reverse)
  rv->sort();
  rv->reverse();

  return rv;
}


/** Get mass point of ball.
 * @param roi ROI to consider
 * @param massPoint contains mass point upon return
 */
void
ReallySimpleClassifier::getMassPointOfBall( ROI *roi, cart_coord_t *massPoint ) {

  unsigned int nrOfOrangePixels;
  nrOfOrangePixels = 0;
  massPoint->x     = 0;
  massPoint->y     = 0;

  // for accessing ROI pixels
  register unsigned int h = 0;
  register unsigned int w = 0;
  // planes
  register unsigned char *yp   = src + (roi->start.y * roi->line_step) + (roi->start.x * roi->pixel_step);
  register unsigned char *up   = YUV422_PLANAR_U_PLANE(src, roi->image_width, roi->image_height)
                                   + ((roi->start.y * roi->line_step) / 2 + (roi->start.x * roi->pixel_step) / 2) ;
  register unsigned char *vp   = YUV422_PLANAR_V_PLANE(src, roi->image_width, roi->image_height)
                                   + ((roi->start.y * roi->line_step) / 2 + (roi->start.x * roi->pixel_step) / 2);
  // line starts
  unsigned char *lyp  = yp; 
  unsigned char *lup  = up;  
  unsigned char *lvp  = vp;  

  color_t color;

  // consider each ROI pixel
  for (h = 0; h < roi->height; ++h) {
    for (w = 0; w < roi->width; w += 2) {
      // classify its color
      color = color_model->determine(*yp++, *up++, *vp++);
      *yp++;
      // ball pixel?
      if (color == C_ORANGE) {
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
