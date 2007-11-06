
/***************************************************************************
 *  trapezium_set.h - Fuzzy Trapezium Set
 *
 *  Generated: Thu May 31 18:36:55 2007
 *  Copyright  2007  Martin Liebenberg
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#ifndef __PLUGINS_NAVIGATOR_FUZZY_TRAPEZIUM_SET_H_
#define __PLUGINS_NAVIGATOR_FUZZY_TRAPEZIUM_SET_H_

#include <plugins/navigator/fuzzy/fuzzy_set.h>

class TrapeziumSet : public FuzzySet
  {
  public:

    TrapeziumSet(double l, double m1, double m2, double r);
    TrapeziumSet(const char* label, double l, double m1, double m2, double r);
    virtual ~TrapeziumSet();

    double membershipGrade(double x);
    void supportBorders(double &left, double &right);
    double getFirstAlphaLevelValue();
    double getMiddleAlphaLevelValue();

  private:

    double m1;
    double m2;
  };


#endif //_TRAPEZIUM_SET_H_
