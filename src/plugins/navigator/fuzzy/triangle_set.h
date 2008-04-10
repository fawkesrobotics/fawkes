
/***************************************************************************
 *  triangle_set.h - Navigator Thread
 *
 *  Generated: Thu May 31 18:36:55 2007
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

#ifndef __PLUGINS_NAVIGATOR_FUZZY_TRIANGLE_SET_H_
#define __PLUGINS_NAVIGATOR_FUZZY_TRIANGLE_SET_H_

#include <plugins/navigator/fuzzy/fuzzy_set.h>

class TriangleSet : public FuzzySet
  {
  public:

    TriangleSet(double l, double m, double r);
    TriangleSet(const char* label, double l, double m, double r);
    virtual ~TriangleSet();

    double membershipGrade(double x);
    void supportBorders(double &left, double &right);
    double getFirstAlphaLevelValue();
    double getMiddleAlphaLevelValue();

  private:

    double m;
  };

#endif
