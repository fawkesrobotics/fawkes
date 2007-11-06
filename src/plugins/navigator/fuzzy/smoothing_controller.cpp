
/***************************************************************************
 *  smoothing_controller.cpp - Fuzzy Smoothing Controller
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

#include <plugins/navigator/fuzzy/smoothing_controller.h>
#include <plugins/navigator/fuzzy/fuzzy_set.h>
#include <plugins/navigator/fuzzy/fuzzy_partition.h>
#include <plugins/navigator/fuzzy/triangle_set.h>


/** @class SmoothingController fuzzy/smoothing_controller.h
 *  This controller assures a smooth drive of the robot.
 *
 * @author Martin Liebenberg
 */
/** @var SmoothingController::inputSets
 * A vector of pointers of FuzzySet. It contains the input fuzzy sets of this controller.
 */
/** @var SmoothingController::outputSets
 * A vector of pointers of FuzzySet. It contains the output fuzzy sets of this controller.
 */
/** @var SmoothingController::conditionSets1
 * A vector of fuzzy sets for the condition of a rule of the rule base.
 */
/** @var SmoothingController::conditionSets2
 * A vector of fuzzy sets for the condition of a rule of the rule base.
 */
/** @var SmoothingController::conditionSets3
 * A vector of fuzzy sets for the condition of a rule of the rule base.
 */

/** Constructor */
SmoothingController::SmoothingController()
{
  inputSets.push_back(new TriangleSet("ZERO", 0, 0, 1));
  inputSets.push_back(new TriangleSet("LITTLE", 0, 1, 1.5));
  inputSets.push_back(new TriangleSet("TOOMUTCH", 1, 3.14, 3.14));

  outputSets.push_back(new TriangleSet("ZERO", 0, 0, 0.1));
  outputSets.push_back(new TriangleSet("MORE", 0, 0.5, 0.8));
  outputSets.push_back(new TriangleSet("FULL", 0.7, 0.8, 0.9));

  FuzzyPartition *input_partition = new FuzzyPartition(&inputSets);
  outputPartition = new FuzzyPartition(&outputSets);
  inputPartitions->push_back(input_partition);

  //the rule base

  conditionSets1 = new std::vector<FuzzySet *>;
  conditionSets1->push_back(inputSets[0]);
  conditionSets2 = new std::vector<FuzzySet *>;
  conditionSets2->push_back(inputSets[1]);
  conditionSets3 = new std::vector<FuzzySet *>;
  conditionSets3->push_back(inputSets[2]);

  addRule(conditionSets1, outputSets[1]);
  addRule(conditionSets2, outputSets[2]);
  addRule(conditionSets3, outputSets[2]);
}


/** Destructor. */
SmoothingController::~SmoothingController()
{
  for(unsigned int i = 0; i < inputSets.size(); i++)
    {
      delete inputSets[i];
    }
  inputSets.clear();

  for(unsigned int i = 0; i < outputSets.size(); i++)
    {
      delete outputSets[i];
    }
  outputSets.clear();

  for(unsigned int i = 0; i < inputPartitions->size(); i++)
    {
      delete inputPartitions->at(i);
    }
  delete conditionSets1;
  delete conditionSets2;
  delete conditionSets3;
  delete outputPartition;
}

/** The controlling method of this controller.
 *   @param difference the difference between the directions of the last and the current
 *                                      drive command
 *   @see FuzzyController::control
 */
double SmoothingController::control(double difference)
{
  std::vector<double> inputValueVector;
  inputValueVector.push_back(difference);

  return MamdaniFuzzyController::control(inputValueVector);
}
