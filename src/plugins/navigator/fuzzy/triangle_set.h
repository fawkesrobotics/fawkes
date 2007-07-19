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
 
#ifndef _TRIANGLE_SET_H_
#define _TRIANGLE_SET_H_

#include "fuzzy_set.h"

class TriangleSet : public FuzzySet 
{
 public:
  
  TriangleSet(double l, double r, double m);
        
  TriangleSet(char* label, double l, double r, double m);
        
  virtual ~TriangleSet();
        
  double membershipGrade(double x);
        
  void supportBorders(double &left, double &right);
        
  double getFirstAlphaLevelValue();
        
  double getMiddleAlphaLevelValue();
        
 private:

  double m;
};

#endif
