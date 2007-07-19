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

#include "fuzzy_controller.h"
#include "fuzzy_partition.h"
#include "fuzzy_set.h"


/** @class FuzzyController fuzzy/fuzzy_controller.h 
 * Absract base class for fuzzy controllers.
 *
 * @author Martin Liebenberg
 *
 * 
 * @fn FuzzyController::fuzzification(std::vector<double> values)
 * The fuzzification of this controller.
 * It transforms the crisp values into fuzzy values.
 * @param values a vector with one crisp value per input partition
 * 
 * @fn FuzzyController::aggregation()
 * The aggregation of this controller.
 * It computes the grade of membership of the conditions of the rules of the rule base.
 * 
 * @fn FuzzyController::interference()
 * The interference of this controller.
 * It computes the grade of membership of the conclusions of the rules of the rule base
 * by means of the results of the aggregation.
 * 
 * @fn FuzzyController:: accumulation()
 * The accumulation of this controller.
 * It joints the conclusion sets and generates one set in the output partition as output
 * of the controlling.
 * 
 * @fn FuzzyController::defuzzification()
 * The defuzzification of this controller.
 * It computes a double value coming from the output partition.
 * 
 */
/** @var FuzzyController::inputPartitions
 *   A vector of pointers to objects of FuzzyPartition.
 *   They are the input partitions which contains the input fuzzy sets.
 */
/** @var FuzzyController::outputPartition
 *   A pointer to a FuzzyPartition.
 *   It is the output partition which contains the output fuzzy sets.
 */
 
/** Empty standard constructor.*/
FuzzyController::FuzzyController()
{
  inputPartitions = new std::vector<FuzzyPartition *>;
}

/** Constructor.
 * @param inputPartitions a vector of pointers to objects of FuzzyPartition
 * @param outputPartition a pointer to a FuzzyPartition
 */
FuzzyController::FuzzyController(       std::vector<FuzzyPartition *>* inputPartitions,
                                        FuzzyPartition* outputPartition)
{
  this->inputPartitions = inputPartitions;
  this->outputPartition = outputPartition;
}

/** Deconstructor. */
FuzzyController::~FuzzyController()
{
  inputPartitions->clear();
  delete inputPartitions;
}

/** The core methode of the fuzzy controller.
 *   It gets some input values and return the output value which controls
 *   the process the controller is for.
 *  @param inputValues a vector of values as input of this controller
 */
double FuzzyController::control(std::vector<double> inputValues)
{
  fuzzification(inputValues);
  aggregation();
  interference();
  accumulation();
  return defuzzification();
}
