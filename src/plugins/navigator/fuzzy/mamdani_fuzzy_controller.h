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

#ifndef MAMDANI_FUZZY_CONTROLLER_H_
#define MAMDANI_FUZZY_CONTROLLER_H_

#include <vector>

#include "fuzzy_controller.h"

class FuzzySet;
class FuzzyPartition;


class MamdaniFuzzyController : public FuzzyController
{
 public:

  MamdaniFuzzyController();
        
  MamdaniFuzzyController(       std::vector<FuzzyPartition *> *inputPartitions,
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
