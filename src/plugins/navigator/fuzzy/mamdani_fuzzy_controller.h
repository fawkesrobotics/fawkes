
/***************************************************************************
 *  mamdani_fuzzy_controller.h - Mamdani Fuzzy Controller
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

#ifndef __PLUGINS_NAVIGATOR_FUZZY_MAMDANI_FUZZY_CONTROLLER_H_
#define __PLUGINS_NAVIGATOR_FUZZY_MAMDANI_FUZZY_CONTROLLER_H_

#include <vector>

#include <plugins/navigator/fuzzy/fuzzy_controller.h>

class FuzzySet;
class FuzzyPartition;


class MamdaniFuzzyController : public FuzzyController
  {
  public:

    MamdaniFuzzyController();
    MamdaniFuzzyController(std::vector<FuzzyPartition *> *inputPartitions,
                           FuzzyPartition* outputPartition );
    virtual ~MamdaniFuzzyController();

    void addRule(std::vector<FuzzySet *> *inputSets, FuzzySet * outputSet);

  private:

    struct Rule
      {
        Rule(std::vector<FuzzySet *> *inputSets, FuzzySet * outputSet)
        {
          this->inputSets = inputSets;
          this->outputSet = outputSet;
        }
        std::vector<FuzzySet *> *inputSets;
        FuzzySet * outputSet;
      };

    void fuzzification(std::vector<double> values);
    void aggregation();
    void interference();
    void accumulation();
    double defuzzification();
    
    std::vector<double> aggregationValues;
    std::vector<Rule *> ruleBase;
  };

#endif /*MAMDANI_FUZZY_CONTROLLER_H_*/
