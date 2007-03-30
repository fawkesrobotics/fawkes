
/***************************************************************************
 *  drawer.cpp - Utility to draw in a buffer
 *
 *  Generated: Wed Feb 08 20:55:38 2006
 *  Copyright  2005-2007  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/draw/drawer.h>
#include <fvutils/color/yuv.h>

#include <cmath>
#include <unistd.h>

/** @class Drawer <fvutils/draw/drawer.h>
 * Draw to an image.
 */

/** Constructor.
 * Default paint color is white.
 */
Drawer::Drawer()
{
  buffer   = NULL;
  color[0] = 255;
  color[1] = 128;
  color[2] = 128;
}

/** Destructor */
Drawer::~Drawer()
{
}


/** Set the buffer to draw to
 * @param buffer buffer to draw to, must be YUV422 planar formatted
 * @param width width of the buffer
 * @param height height of the buffer
 */
void
Drawer::setBuffer(unsigned char *buffer,
		  unsigned int width, unsigned int height)
{
  this->buffer     = buffer;
  this->width      = width;
  this->height     = height;
}


/** Set drawing color.
 * @param y Y component of YUV drawing color
 * @param u U component of YUV drawing color
 * @param v V component of YUV drawing color
 */
void
Drawer::setColor(unsigned char y, unsigned char u, unsigned char v)
{
  color[0] = y;
  color[1] = u;
  color[2] = v;
}


/** Draw circle.
 * Draws a circle at the given center point and with the given radius.
 * @param center_x x coordinate of circle center
 * @param center_y y coordinate of circle center
 * @param radius radius of circle
 */
void
Drawer::drawCircle(int center_x, int center_y, unsigned int radius)
{

  if (buffer == NULL) return;

  unsigned int x  = 0,
               y  = radius,
               r2 = radius * radius;

  unsigned char *up = YUV422_PLANAR_U_PLANE(buffer, width, height);
  unsigned char *vp = YUV422_PLANAR_V_PLANE(buffer, width, height);

  unsigned int x_tmp, y_tmp, ind_tmp;

  while (x < y) {

    x_tmp = center_x + x;
    y_tmp = center_y + y;
    if ( (x_tmp < width) && (y_tmp < height) ) {
      ind_tmp = y_tmp * width + x_tmp;
      buffer[ind_tmp]   = color[0];
      ind_tmp /= 2;
      up[ind_tmp] = color[1];
      vp[ind_tmp] = color[2];
    }
 
    x_tmp = center_x - x;
    y_tmp = center_y + y;
    if ( (x_tmp < width) && (y_tmp < height) ) {
      ind_tmp = y_tmp * width + x_tmp;
      buffer[ind_tmp]   = color[0];
      ind_tmp /= 2;
      up[ind_tmp] = color[1];
      vp[ind_tmp] = color[2];
    }
 
    x_tmp = center_x + y;
    y_tmp = center_y + x;
    if ( (x_tmp < width) && (y_tmp < height) ) {
      ind_tmp = y_tmp * width + x_tmp;
      buffer[ind_tmp]   = color[0];
      ind_tmp /= 2;
      up[ind_tmp] = color[1];
      vp[ind_tmp] = color[2];
    }
 
    x_tmp = center_x - y;
    y_tmp = center_y + x;
    if ( (x_tmp < width) && (y_tmp < height) ) {
      ind_tmp = y_tmp * width + x_tmp;
      buffer[ind_tmp]   = color[0];
      ind_tmp /= 2;
      up[ind_tmp] = color[1];
      vp[ind_tmp] = color[2];
    }
 
    x_tmp = center_x + x;
    y_tmp = center_y - y;
    if ( (x_tmp < width) && (y_tmp < height) ) {
      ind_tmp = y_tmp * width + x_tmp;
      buffer[ind_tmp]   = color[0];
      ind_tmp /= 2;
      up[ind_tmp] = color[1];
      vp[ind_tmp] = color[2];
    }
    
    x_tmp = center_x - x;
    y_tmp = center_y - y;
    if ( (x_tmp < width) && (y_tmp < height)) {
      ind_tmp = y_tmp * width + x_tmp;
      buffer[ind_tmp]   = color[0];
      ind_tmp /= 2;
      up[ind_tmp] = color[1];
      vp[ind_tmp] = color[2];
    }
 
    x_tmp = center_x + y;
    y_tmp = center_y - x;
    if ( (x_tmp < width) && (y_tmp < height)) {
      ind_tmp = y_tmp * width + x_tmp;
      buffer[ind_tmp]   = color[0];
      ind_tmp /= 2;
      up[ind_tmp] = color[1];
      vp[ind_tmp] = color[2];
    }
    
    x_tmp = center_x - y;
    y_tmp = center_y - x;
    if ( (x_tmp < width) && (y_tmp < height) ) {
      ind_tmp = y_tmp * width + x_tmp;
      buffer[ind_tmp]   = color[0];
      ind_tmp /= 2;
      up[ind_tmp] = color[1];
      vp[ind_tmp] = color[2];
    }
    
    ++x;
    y=(int)(sqrt((float)(r2 - x * x))+0.5);
  }

}


/** Draw rectangle.
 * @param x x coordinate of rectangle's upper left corner
 * @param y y coordinate of rectangle's upper left corner
 * @param w width of rectangle from x to the right
 * @param h height of rectangle from y to the bottom
 */
void
Drawer::drawRectangle(unsigned int x, unsigned int y,
		      unsigned int w, unsigned int h)
{

  unsigned char *up = YUV422_PLANAR_U_PLANE(buffer, width, height);
  unsigned char *vp = YUV422_PLANAR_V_PLANE(buffer, width, height);

  // horizontal line at top
  for (unsigned int i = x; i < x + w; ++i) {
    if ( i < width ) {
      buffer[ y * width + i ]   = color[0];
      up[ (y * width + i) / 2 ] = color[1];
      vp[ (y * width + i) / 2 ] = color[2];
    } else {
      break;
    }
  }

  // left and right
  for (unsigned int i = y; i < y + h; ++i) {
    // left
    buffer[ i * width + x ]   = color[0];
    up[ (i * width + x) / 2 ] = color[1];
    vp[ (i * width + x) / 2 ] = color[2];

    if ( (x + w) < width ) {
      // right
      buffer[ i * width + x + w ]   = color[0];
      up[ (i * width + x + w) / 2 ] = color[1];
      vp[ (i * width + x + w) / 2 ] = color[2];
    }
  }

  // horizontal line at bottom
  for (unsigned int i = x; i < x + w; ++i) {
    if ( i < width ) {
      buffer[ (y + h) * width + i ]   = color[0];
      up[ ((y + h) * width + i) / 2 ] = color[1];
      vp[ ((y + h) * width + i) / 2 ] = color[2];
    } else {
      break;
    }
  }

}


/** Draw inverted rectangle.
 * This draws a rectangle but instead of using the draw color it is drawn
 * in the inverted color of the pixel where it is drawn.
 * @param x x coordinate of rectangle's upper left corner
 * @param y y coordinate of rectangle's upper left corner
 * @param w width of rectangle from x to the right
 * @param h height of rectangle from y to the bottom
 */
void
Drawer::drawRectangleInverted(unsigned int x, unsigned int y,
			      unsigned int w, unsigned int h)
{

  unsigned int ind = 0;

  // horizontal line at top
  for (unsigned int i = x; i < x + w; ++i) {
    if ( i < width ) {
      ind = y * width + i;
      buffer[ind]   = 255 - buffer[ind];
    } else {
      break;
    }
  }

  // left and right
  for (unsigned int i = y; i < y + h; ++i) {
    // left
    ind = i * width + x;
    buffer[ind]   = 255 - buffer[ind];

    if ( (x + w) < width ) {
      // right
      ind += w;
      buffer[ind]   = 255 - buffer[ind];
    }
  }

  // horizontal line at bottom
  for (unsigned int i = x; i < x + w; ++i) {
    if ( i < width ) {
      buffer[ind]   = 255 - buffer[ind];
    } else {
      break;
    }
  }

}


/** Draw point.
 * @param x x coordinate of point
 * @param y y coordinate of point
 */
void
Drawer::drawPoint(unsigned int x, unsigned int y)
{
  if ( x > width) return;
  if ( y > height) return;

  unsigned char *up = YUV422_PLANAR_U_PLANE(buffer, width, height);
  unsigned char *vp = YUV422_PLANAR_V_PLANE(buffer, width, height);

  buffer[ y * width + x ]   = color[0];
  up[ (y * width + x) / 2 ] = color[1];
  vp[ (y * width + x) / 2 ] = color[2];
}


/** Color the given point.
 * This will leave the Y-component of the given pixel unchanged and will
 * just set the U and V components. This can be used to keep a little bit
 * of original image information but marking special regions.
 * @param x x coordinate of point
 * @param y y coordinate of point
 */
void
Drawer::colorPoint(unsigned int x, unsigned int y)
{
  if ( x > width) return;
  if ( y > height) return;

  unsigned char *up = YUV422_PLANAR_U_PLANE(buffer, width, height);
  unsigned char *vp = YUV422_PLANAR_V_PLANE(buffer, width, height);

  up[ (y * width + x) / 2 ] = color[1];
  vp[ (y * width + x) / 2 ] = color[2];
}


/** Draw line.
 * Standard Bresenham in all directions. For in-depth information
 * have a look at http://de.wikipedia.org/wiki/Bresenham-Algorithmus
 * @param x_start x coordinate of start point
 * @param y_start y coordinate of start point
 * @param x_end x coordinate of end point
 * @param y_end y coordinate of end point
 */
void
Drawer::drawLine(unsigned int x_start, unsigned int y_start,
		 unsigned int x_end, unsigned int y_end)
{
  /* heavily inspired by an article on German Wikipedia about
   * Bresenham's algorithm, confer
   * http://de.wikipedia.org/wiki/Bresenham-Algorithmus
   */


  int x, y, dist, xerr, yerr, dx, dy, incx, incy;
  bool was_inside_image = false;

  unsigned char *up = YUV422_PLANAR_U_PLANE(buffer, width, height);
  unsigned char *vp = YUV422_PLANAR_V_PLANE(buffer, width, height);
 
  // calculate distance in both directions
  dx = x_end - x_start;
  dy = y_end - y_start;
 
  // Calculate sign of the increment
  if(dx < 0) {
    incx = -1;
    dx = -dx;
  } else {
    incx = dx ? 1 : 0;
  }
     
  if(dy < 0) {
    incy = -1;
    dy = -dy;
  } else {
    incy = dy ? 1 : 0;
  }
 
  // check which distance is larger
  dist = (dx > dy) ? dx : dy;
 
  // Initialize for loops
  x = x_start;
  y = y_start;
  xerr = dx;
  yerr = dy;
     
  /* Calculate and draw pixels */
  for(int t = 0; t < dist; ++t) {
    if ( ((unsigned int)x < width) && ((unsigned int)y < height) ) {
      if ( (x >= 0) && (y >= 0) ) {
	was_inside_image = true;
	buffer[ y * width + x ]   = color[0];
	up[ (y * width + x) / 2 ] = color[1];
	vp[ (y * width + x) / 2 ] = color[2];
      }
    } else {
      if ( was_inside_image ) {
	break;
      }
    }

    xerr += dx;
    yerr += dy;
     
    if(xerr > dist) {
      xerr -= dist;
      x += incx;
    }
     
    if(yerr>dist) {
      yerr -= dist;
      y += incy;
    }
  }
     
  if ( (x_end < width) && (y_end < height) ) {
    buffer[ y_end * width + x_end ]   = color[0];
    up[ (y_end * width + x_end) / 2 ] = color[1];
    vp[ (y_end * width + x_end) / 2 ] = color[2];
  }
     
}
