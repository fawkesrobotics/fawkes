/***************************************************************************
 *  goal.cpp -
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

#include "goalAction.h"

#include <list>
#include <map>
#include <string>
using namespace std;

Param::Param(const string a_name, const string a_value)
{
	name  = a_name;
	value = a_value;
}

Param::Param(const string a_name, const string a_type, const string a_value)
{
	name  = a_name;
	type  = a_type;
	value = a_value;
}

std::string
Param::getParamString()
{
	string temp = name + '#' + value;
	return temp;
}

GoalAction::GoalAction(const string a_classname)
{
	class_name = a_classname;
}

GoalAction::GoalAction(const string a_classname, const string a_id)
{
	class_name = a_classname;
	id         = a_id;
}

void
GoalAction::setParams(const list<Param> aParams)
{
	params = aParams;
}

void
GoalAction::addParam(const Param aParam)
{
	params.push_back(aParam);
}

std::string
GoalAction::getClass()
{
	return class_name;
}
std::string
GoalAction::getId()
{
	return id;
}
std::list<Param> *
GoalAction::getParams()
{
	return &params;
}

list<Param>
GoalAction::getSortedParams()
{
	params.sort();
	return params;
}

string
GoalAction::getParamsString()
{
	params.sort();
	string paramsString = "";

	for (Param &p : params) {
		paramsString +="#"+ p.getParamString();
	}

	return paramsString;
}

string
GoalAction::getGoalString()
{
	string goalString = class_name + getParamsString();
	return goalString;
}

/*const
std::string 
GoalActionUtil::getGoalIdByString(std::vector<GoalAction> goals, std::string goal_str)
{
    for(GoalAction g : goals)
    {
        if(g.getGoalString() == goal_str)
        {
            return g.getId();
        }
    }
    return "";
}*/
