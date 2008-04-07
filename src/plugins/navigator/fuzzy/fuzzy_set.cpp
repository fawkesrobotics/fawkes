
/***************************************************************************
 *  fuzzy_set.cpp - Fuzzy Set
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
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include <plugins/navigator/fuzzy/fuzzy_set.h>
#include <limits.h>
#include <cstring>
#include <cstdlib>

long FuzzySet::idCounter = 0;

/** @class FuzzySet fuzzy_set.h
 * Abstract base class for fuzzy sets.
 *
 * @author Martin Liebenberg
 * 
 *
 * @fn FuzzySet::membershipGrade(double x)
 * Returns the grade of membership of x in this set.
 * @param x a value out of the univers of this set
 * @return double, between 0 and 1, containing the grade of membership of x in this set
 * 
 * @fn FuzzySet::supportBorders(double &left, double &right)
 * Supports the borders of this set.
 * @param left reference which gets the left border
 * @param right reference which gets the right border
 *
 * @fn FuzzySet::getFirstAlphaLevelValue()
 * Returns the first value of the alpha-level set of this fuzzy set.
 * @return double containing the first value of the alpha-level set of this fuzzy set
 *
 * @fn FuzzySet::getMiddleAlphaLevelValue()
 * Returns the middle value of the alpha-level set of this fuzzy set.
 * @return double containing the middle value of the alpha-level set of this fuzzy set
 */
/** @var FuzzySet::label
 *   A C string containing a lettering for this set.
 */
/** @var FuzzySet::alphaLevel
 * The maximum of the alpha-level set of this fuzzy set.
 */
/** @var FuzzySet::l
 * Left border of the set.
 */
/** @var FuzzySet::r
 * Right border of the set.
 */
/** @var FuzzySet::idCounter
 * Counts the ids to be sure they are unique.
 */
/** @var FuzzySet::id
 * The unique id of this fuzzy set.
 */

/** Constructor. */
FuzzySet::FuzzySet()
{
  id = idCounter;
  if(id < LONG_MAX)
    {
      idCounter++;
    }
  else
    {
      idCounter = 0;
    }
  alphaLevel = 0;
  label = 0;
}

/** Destructor. */
FuzzySet::~FuzzySet()
{
  if(label != 0)
    {
      free(label);
    }
}

/** Sets a lettering for this fuzzy set.
 * @param label new label
 */
void FuzzySet::setLabel(const char* label)
{
  if(this->label != 0)
    {
      free(this->label);
    }
  this->label = strdup(label);
}

/** Returns the lettering for this fuzzy set.
 *  @return a constant char array containing the lettering for this fuzzy set
 */
const char* FuzzySet::getLabel()
{
  if(label != 0)
    {
      return label;
    }
  else
    {
      return "";
    }
}

/** Sets the maximum of the alpha-level set of this fuzzy set.
 *   Alpha has to lie between 0 and 1.
 *  @param alpha the alpha level
 */
void FuzzySet::setAlphaLevel(double alpha)
{
  alphaLevel = alpha;
}

/** Returns the maximum of the alpha-level set of this fuzzy set.
 *   Alpha lies between 0 and 1.
 * @return double containing the maximum of the alpha-level set of this fuzzy set
 */
double FuzzySet::getAlphaLevel()
{
  return alphaLevel;
}


/** Returns the unique id of this fuzzy set.
 * @return long containing the unique id of this fuzzy set
 */
long FuzzySet::getId()
{
  return id;
}
