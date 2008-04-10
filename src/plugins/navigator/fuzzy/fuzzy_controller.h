
/***************************************************************************
 *  fuzzy_controller.h - Fuzzy Controller
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
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_NAVIGATOR_FUZZY_FUZZY_CONTROLLER_H_
#define __PLUGINS_NAVIGATOR_FUZZY_FUZZY_CONTROLLER_H_

#include <vector>

class FuzzyPartition;


class FuzzyController
  {
  public:

    FuzzyController();
    FuzzyController(      std::vector<FuzzyPartition *> *inputPartitions,
                          FuzzyPartition* outputPartition);
    virtual ~FuzzyController();
    
    double control(std::vector<double> inputValues);

  protected:

    virtual void fuzzification(std::vector<double> values) = 0;
    virtual void aggregation() = 0;
    virtual void interference() = 0;
    virtual void accumulation() = 0;
    virtual double defuzzification() = 0;

    std::vector<FuzzyPartition *> *inputPartitions;
    FuzzyPartition *outputPartition;
  };

#endif /*FUZZY_CONTROLLER_H_*/
