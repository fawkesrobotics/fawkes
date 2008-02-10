
/***************************************************************************
 *  drawer.h - Drawer allows to draw arbitrarily in a buffer
 *
 *  Generated: Wed Feb 08 20:30:00 2006
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

#ifndef __FIREVISION_FVUTILS_DRAWER_H_
#define __FIREVISION_FVUTILS_DRAWER_H_

class Drawer {

 public:
  Drawer();
  ~Drawer();

  void drawCircle(int center_x, int center_y, unsigned int radius);

  void drawRectangle(unsigned int x, unsigned int y,
		     unsigned int w, unsigned int h);

  void drawRectangleInverted(unsigned int x, unsigned int y,
			     unsigned int w, unsigned int h);

  void drawPoint(unsigned int x, unsigned int y);
  void colorPoint(unsigned int x, unsigned int y);
  void drawLine(unsigned int x_start, unsigned int y_start,
		unsigned int x_end, unsigned int y_end);

  void setBuffer(unsigned char *buffer,
		 unsigned int width, unsigned int height);

  void setColor(unsigned char y, unsigned char u, unsigned char v);

 private:
  unsigned char  *buffer;
  unsigned int    width;
  unsigned int    height;
  unsigned char   color[3];

};


#endif
