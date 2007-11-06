
/***************************************************************************
 *  fuzzy_partition.cpp - Fuzzy Partition
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

#include <plugins/navigator/fuzzy/fuzzy_partition.h>
#include <plugins/navigator/fuzzy/fuzzy_set.h>

#include <float.h>

/** @class FuzzyPartition fuzzy/fuzzy_partition.h
 * Fuzzy Partition.
 * This class provides a fuzzy partition for a rule base of a fuzzy controller.
 * Therfore there are methods for defuzzification.
 *
 * @author Martin Liebenberg
 */
/** @var FuzzyPartition::sets
 * The fuzzy sets of this partition.
 */
/** @var FuzzyPartition::minL
 * The minimum left value out of this partition.
 */
/** @var FuzzyPartition::maxR
 * The maximum right value out of this partition.
 */

/** Standard constructor.
 */
FuzzyPartition::FuzzyPartition()
{
  minL = 0;
  maxR = 0;
}

/** Constructor.
 * @param sets the sets constituting this partition
 */
FuzzyPartition::FuzzyPartition(std::vector<FuzzySet *> *sets)
{
  this->sets = sets;
  minL = DBL_MAX;
  maxR = 0;
  for(unsigned int j = 0; j < sets->size(); j++)
    {
      double l;
      double r;
      sets->at(j)->supportBorders(l, r);
      if(minL > l)
        minL = l;
      if(maxR < r)
        maxR = r;
    }
  if(minL == DBL_MAX)
    minL = 0;

}

/** Destructor. */
FuzzyPartition::~FuzzyPartition()
{}

/** Returns the distance between the minimum left value
 *   and the maximum right value out of this partition.
 *  @return the distance between the minimum left value and the maximum right value out of this partition
 */
double FuzzyPartition::getSupportRange()
{
  return maxR - minL;
}

/** Returns the maximum right value out of this partition.
 *  @return the maximum right value out of this partition
 */
double FuzzyPartition::getMaxR()
{
  return maxR;
}

/** Returns the minimum left value out of this partition.
 *  @return the minimum left value out of this partition
 */
double FuzzyPartition::getMinL()
{
  return minL;
}


/** Returns the first (left) value out of the alpha-level set
 *    with the highest alpha-level. This is needed for defuzzification.
 *  @return the first (left) value out of the alpha-level set 
 *      with the highest alpha-level
 */
double FuzzyPartition::getfirstMax()
{
  double value = 0;
  double maxAlphaLevel = 0;
  for(unsigned int i = 0; i < sets->size(); i++)
    {
      double alpha = sets->at(i)->getAlphaLevel();

      if(maxAlphaLevel < alpha)
        {
          maxAlphaLevel = alpha;
          value = sets->at(i)->getFirstAlphaLevelValue();
        }
    }
  return value;
}

/** Returns the middle value out of the alpha-level set
 *    with the highest alpha-level. This is needed for defuzzification.
 *  @return the middle value out of the alpha-level set
 *    with the highest alpha-level
 */
double FuzzyPartition::getMiddleMax()
{
  double value = 0;
  double maxAlphaLevel = 0;
  for(unsigned int i = 0; i < sets->size(); i++)
    {
      double alpha = sets->at(i)->getAlphaLevel();

      if(maxAlphaLevel < alpha)
        {
          maxAlphaLevel = alpha;
          value = sets->at(i)->getMiddleAlphaLevelValue();
        }
    }
  return value;
}

/** Returns a kind of weighted average of all maxima of the alpha-level set
 *   of all fuzzy sets out of this partition. 
 *   It is a continuous defuzzification in contrast to the other two defuzzification 
 *   methods in this class.
 *   It is comparable whith the defuzzifications using the centroid or
 *   center-of-gravity method. 
 *  @return a average maximum of all maxima of the sets of this partition
 */
double FuzzyPartition::getWeightedAverageMax()
{
  double middelMaximumValue = 0;
  unsigned int max = 0;
  double maxAlphaLevel = 0;
  for(unsigned int i = 0; i < sets->size(); i++)
    {
      double alpha = sets->at(i)->getAlphaLevel();

      if(maxAlphaLevel < alpha)
        {
          maxAlphaLevel = alpha;
          middelMaximumValue = sets->at(i)->getMiddleAlphaLevelValue();
          max = i;
        }
    }

  double sum = 0;

  for(unsigned int i = 0; i < sets->size(); i++)
    {
      if(i != max && maxAlphaLevel != 0)
        sum += ((sets->at(i)->getMiddleAlphaLevelValue()
                 - sets->at(max)->getMiddleAlphaLevelValue())
                * (sets->at(i)->getAlphaLevel() / maxAlphaLevel));

    }

  return middelMaximumValue + sum / 2.;
}

/**Returns the fuzzy sets of this partition.
 *  @return a vector containing the fuzzy sets of this partitio
 */
std::vector<FuzzySet *> *FuzzyPartition::getFuzzySets()
{
  return sets;
}

