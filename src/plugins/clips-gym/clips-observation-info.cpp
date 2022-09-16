#include "clips-observation-info.h"

#include <string>
#include <map>
#include <list>


ClipsObservationInfo::ClipsObservationInfo(std::map<std::string,int> a_observation_space)
{
    observation_space = a_observation_space;
}



