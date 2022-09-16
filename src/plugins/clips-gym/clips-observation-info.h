
#include <string>
#include <map>
#include <list>


class ClipsObservationInfo
{
public:
    ClipsObservationInfo(std::map<std::string,int> a_observation_space);
    //virtual ~ClipsObservationInfo();

    std::list<int> observation;
    int reward;
    bool done;
    std::string info;

private:
    std::map<std::string,int> observation_space;

};


