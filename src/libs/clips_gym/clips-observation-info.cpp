/***************************************************************************
 *  clips-observation-info.cpp -
 *
 *  Created:
 *  Copyright
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

#include "clips-observation-info.h"

#include <list>
#include <map>
#include <string>

ClipsObservationInfo::ClipsObservationInfo() //std::map<std::string, int> a_observation_space)
{
	//observation_space = a_observation_space;
}

ClipsObservationInfo::ClipsObservationInfo(
  std::string a_observation) //std::map<std::string, int> a_observation_space)
{
	observation = a_observation;
}