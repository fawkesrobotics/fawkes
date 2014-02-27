#include <vector>
#include <string>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

class EclipsePath
{
  private:
    EclipsePath();
  public:
    static void create_initial_object();
    static EclipsePath* instance();
    void add_path(std::string path);
    void add_path_check(std::string path);
    std::string locate_file(std::string filename);
    void add_regex(boost::regex re, std::string str);
    void apply_regexes();
    void print_all_paths();

  private:
    static EclipsePath* m_instance;
  
  public: 
	std::vector<std::string> paths;
 	  std::map<boost::regex,std::string> regexes;
};


extern "C" int p_locate_file(...);
