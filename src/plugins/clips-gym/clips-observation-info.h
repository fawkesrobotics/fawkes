/***************************************************************************
 *  clips-observation-info.h -
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

#include <list>
#include <map>
#include <string>

class ClipsObservationInfo
{
public:
	ClipsObservationInfo(std::map<std::string, int> a_observation_space);
	//virtual ~ClipsObservationInfo();

	std::list<int> observation;
	int            reward;
	bool           done;
	std::string    info;

private:
	std::map<std::string, int> observation_space;
};
