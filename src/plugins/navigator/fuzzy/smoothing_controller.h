
/***************************************************************************
 *  smoothing_controller.h - Fuzzy Smoothing Controller
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

#ifndef __PLUGINS_NAVIGATOR_FUZZY_SMOOTHING_CONTROLLER_H_
#define __PLUGINS_NAVIGATOR_FUZZY_SMOOTHING_CONTROLLER_H_

#include <vector>

#include <plugins/navigator/fuzzy/mamdani_fuzzy_controller.h>

class SmoothingController : MamdaniFuzzyController
  {
  public:

    SmoothingController();
    virtual ~SmoothingController();

    double control(double x);

  private:

    std::vector<FuzzySet *> inputSets;
    std::vector<FuzzySet *> outputSets;
    std::vector<FuzzySet *> *conditionSets1;
    std::vector<FuzzySet *> *conditionSets2;
    std::vector<FuzzySet *> *conditionSets3;
  };

#endif //_SMOOTHING_CONTROLLER_H_
