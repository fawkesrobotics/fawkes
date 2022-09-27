#include "clips-observation-info.h"

#include <list>
#include <map>
#include <string>

ClipsObservationInfo::ClipsObservationInfo(std::map<std::string, int> a_observation_space)
{
	observation_space = a_observation_space;
}
