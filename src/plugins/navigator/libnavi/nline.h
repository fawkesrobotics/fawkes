
/***************************************************************************
 *  nline.h - Navigator Line
 *
 *  Generated: Tue Jun 05 13:58:18 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
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

#ifndef __NAVIGATOR_LIBNAVI_NLINE_H_
#define __NAVIGATOR_LIBNAVI_NLINE_H_

#include <plugins/navigator/libnavi/npoint.h>

class NLine
{
 public:
  NLine();

  NLine(NPoint *p1, NPoint *p2);
  NLine(double x1, double y1, double x2, double y2);
  ~NLine();

  NPoint *p1;
  NPoint *p2;
};

#endif
