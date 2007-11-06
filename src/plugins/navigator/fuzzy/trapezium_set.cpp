
/***************************************************************************
 *  trapezium_set.cpp - Fuzzy Trapezium Set
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
 
#include <plugins/navigator/fuzzy/trapezium_set.h>
#include <cstring>

 
/** @class TrapeziumSet trapezium_set.h 
 * Fuzzy Trapezium Set.
 * This class provides a trapezium shaped fuzzy set.
 *
 * @author Martin Liebenberg
 */
/** @var TrapeziumSet::m1
 * First value of the maximum.
 */
/** @var TrapeziumSet::m2 
 * Last value of the maximum.
 */
 
/** Constructor.
 * @param l left border of the set
 * @param m1 first value of the maximum
 * @param m2 last value of the maximum
 * @param r right border of the set
 */
TrapeziumSet::TrapeziumSet(double l, double m1, double m2, double r)
{
  this->l = l;
  this->r = r;
  this->m1 = m1;
  this->m2 = m2;
  alphaLevel = 0;
}
 
/** Constructor.
 * @param label a lettering for the set
 * @param l left border of the set
 * @param m1 first value of the maximum
 * @param m2 last value of the maximum
 * @param r right border of the set
 */
TrapeziumSet::TrapeziumSet(const char* label, double l, double m1, double m2, double r)
{
  this->label = strdup(label);
  this->l = l;
  this->r = r;
  this->m1 = m1;
  this->m2 = m2;
  alphaLevel = 0;
}
 
/** Destructor. */
TrapeziumSet::~TrapeziumSet()
{
}
 
/** Returns the grade of membership of x in this set.
 * @param x a value out of the univers of this set
 * @return double, between 0 and 1, containing the grade of membership of x in this set
 */
double TrapeziumSet::membershipGrade(double x)
{
  if(x < l)
    return 0;
  else if(l < x && x < m1)
    return (x - l) / (m1 - l);
  else if(m1 <= x && x <= m2)
    return 1;
  else if(m2 < x && x < r)
    return ((m2 - x) / (r - m2)) + 1;
  else
    return 0;
}

  
/** Supports the borders of this set.
 * @param left reference which gets the left border
 * @param right reference which gets the right border
 */
void TrapeziumSet::supportBorders(double &left, double &right)
{
  left = l;
  right = r;
}
 
/** Returns the first value of the alpha-level set of this fuzzy set.
 * @return double containing the first value of the alpha-level set of this fuzzy set
 */
double TrapeziumSet::getFirstAlphaLevelValue()
{
  return alphaLevel * (m1 - l) + l;
}
 
/** Returns the middle value of the alpha-level set of this fuzzy set.
 * @return double containing the middle value of the alpha-level set of this fuzzy set
 */
double TrapeziumSet::getMiddleAlphaLevelValue()
{
  return ( -((alphaLevel - 1) * (r - m2) - m2) + (alphaLevel * (m1 - l) + l)) / 2. ;
}
