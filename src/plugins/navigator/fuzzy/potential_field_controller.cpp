
/***************************************************************************
 *  potential_field_controller.cpp - Fuzzy Potential Field Controller
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

#include <plugins/navigator/fuzzy/potential_field_controller.h>
#include <plugins/navigator/fuzzy/fuzzy_partition.h>
#include <plugins/navigator/fuzzy/triangle_set.h>
#include <plugins/navigator/fuzzy/trapezium_set.h>
#include <plugins/navigator/fuzzy/fuzzy_set.h>


/** @class PotentialFieldController fuzzy/potential_field_controller.h
 *  This controller realises a kind of potential field method for obstacle avoidance.
 *  The nearer an obstacle appears the more the robot is repelled by this obstacle.
 *
 * @author Martin Liebenberg
 */
/** @var PotentialFieldController::inputSets
 * A vector of pointers of FuzzySet. It contains the input fuzzy sets of this controller.
 */
/** @var PotentialFieldController::outputSets
 * A vector of pointers of FuzzySet. It contains the output fuzzy sets of this controller.
 */
/** @var PotentialFieldController::conditionSets1
 * A vector of fuzzy sets for the condition of a rule of the rule base.
 */
/** @var PotentialFieldController::conditionSets2
 * A vector of fuzzy sets for the condition of a rule of the rule base.
 */
/** @var PotentialFieldController::conditionSets3
 * A vector of fuzzy sets for the condition of a rule of the rule base.
 */

/** Contructor.
 *  @param robot_width at present not in use 
 */
PotentialFieldController::PotentialFieldController(double robot_width)
{
  inputSets.push_back(new TrapeziumSet("OK", 0.10, 0.30, 100000, 100000));
  inputSets.push_back(new TriangleSet("NEAR", 0.05, 0.10, 0.15));
  inputSets.push_back(new TrapeziumSet("TOO_NEAR", 0, 0, 0.1, 0.5));

  outputSets.push_back(new TriangleSet("ZERO", 0, 0, 1));
  outputSets.push_back(new TriangleSet("LOW", 0, 1.5, 2.5));
  outputSets.push_back(new TriangleSet("HIGH", 1.25, 2.5, 5));

  FuzzyPartition *input_partition = new FuzzyPartition(&inputSets);
  inputPartitions->push_back(input_partition);
  outputPartition = new FuzzyPartition(&outputSets);

  //the rule base
  conditionSets1 = new std::vector<FuzzySet *>;
  conditionSets1->push_back(inputSets[0]);
  conditionSets2 = new std::vector<FuzzySet *>;
  conditionSets2->push_back(inputSets[1]);
  conditionSets3 = new std::vector<FuzzySet *>;
  conditionSets3->push_back(inputSets[2]);

  addRule(conditionSets1, outputSets[0]);
  addRule(conditionSets2, outputSets[1]);
  addRule(conditionSets3, outputSets[2]);
}

/** Destructor. */
PotentialFieldController::~PotentialFieldController()
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
  //     outputSets.clear();
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
 *   @param distance the distance between the robot and an obstacle
 *   @see FuzzyController::control
 */
double PotentialFieldController::control(double distance)
{
  std::vector<double> inputValueVector;
  inputValueVector.push_back(distance);

  return MamdaniFuzzyController::control(inputValueVector);
}
