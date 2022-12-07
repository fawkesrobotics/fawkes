/***************************************************************************
 *  goal.h -
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
#include <vector>
#include <string>

class Param
{
public:
    Param(const std::string name,const std::string value);

    Param(const std::string name,const std::string type,const std::string value);

    std::string getParamString();

    bool operator < (const Param& param) const
    {
        return name < param.name;
    }

    std::string name;
    std::string type;
    std::string value;
private:
    


};


class GoalAction
{
public:
	//GoalAction(); 
    GoalAction(const std::string classname);
	GoalAction(const std::string classname, const std::string id);
    void setParams(const std::list<Param> aParams);
    void addParam(const Param aParam);

    std::string getClass();
    std::string getId();
    std::list<Param>* getParams();

	
    std::string getGoalString();
    std::string getParamsString();
    std::list<Param> getSortedParams();
    
   
private:
	std::string class_name;
	std::string id="";
    std::list<Param> params={};
};

namespace GoalActionUtil
{
     const
    std::string getGoalIdByString(std::vector<GoalAction> goals, std::string goal_str);

}