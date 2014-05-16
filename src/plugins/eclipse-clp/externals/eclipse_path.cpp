#include "eclipse_path.h"
#include <iterator>
#include <iostream>
#include <core/exception.h>
#include <eclipseclass.h>

using namespace boost::filesystem;
using namespace fawkes;

EclipsePath* EclipsePath::m_instance = NULL;


/** Constructor. */
EclipsePath::EclipsePath()
{
	if (m_instance == NULL){
		m_instance = this;
   	}else{
		//throw Exception( "There is already an instance of type EclipsePath instantiated" );
	}
	
}

void
EclipsePath::create_initial_object(){
	m_instance = new EclipsePath();
	m_instance->add_regex(boost::regex("@basedir@"), BASEDIR);
	m_instance->add_regex(boost::regex("@confdir@"), CONFDIR);
	
	m_instance->add_path("@basedir@/");
	m_instance->apply_regexes();
	std::string fawkes_path = m_instance->locate_file("fawkes");
	m_instance->paths.clear(); // remove basedir from paths
	//std::cout << "searched for fawkes folder" << fawkes_path << '\n';
	if (not fawkes_path.empty()){
		//std::cout << "found fawkes folder!\n";
		m_instance->add_regex(boost::regex("@fawkesdir@"), fawkes_path);
	}else{
		// for convinience, if only fawkes exists, replace @fawkesdir@ with BASEDIR
		m_instance->add_regex(boost::regex("@fawkesdir@"), BASEDIR);
	}
}

EclipsePath* EclipsePath::instance()
{
	if ( !m_instance )
	{ throw Exception( "No instance of type EclipsePath initialised" ); }

	return m_instance;
}



void
EclipsePath::add_path(std::string path)
{
	//std::cout << "add path: " << path << '\n';
	paths.push_back(path);
}


void
EclipsePath::add_path_check(std::string path)
{
	instance()->add_path(path);
	instance()->apply_regexes();
}

std::string
EclipsePath::locate_file(std::string filename)
{
	if (paths.empty()){
		return "";
	}
	//std::cout << "locate file: " << filename << '\n';
	for (std::vector<std::string>::iterator it = paths.begin(); it != paths.end(); ++it){
		path p (*it);
		p /= filename;
		//std::cout << "locate file: created path for:" << p.native() << '\n' ;
		try
		{
			if (exists(p))
			{
				//std::cout << "found file " << filename << " at:" << '\n'; 
				return p.native();
			}
		}
		catch (const filesystem_error& ex)
		{
			throw Exception( "Filesystem error" );
		}
		
	}
	return "";
}

void
EclipsePath::apply_regexes()
{
	int i;
	std::vector<std::string>::iterator it;
	for (i = 0, it = paths.begin(); it != paths.end(); ++it, i++){
		for (std::map<boost::regex,std::string>::iterator re=regexes.begin(); re!=regexes.end(); ++re){
		std::string result = boost::regex_replace(*it, re->first, re->second);
		//std::cout << "path: " << paths[i] << '\n'; 
		paths[i]=result;
		//std::cout << "applying: " << re->first << "=>" << re->second << "\nregex result:" << result << '\n';

	}
	}
}

void
EclipsePath::print_all_paths()
{
	for ( std::vector<std::string>::iterator it = paths.begin(); it != paths.end(); ++it){
              //std::cout << *it << '\n';
	}
}

void
EclipsePath::add_regex(boost::regex re, std::string str)
{
	regexes.insert( std::pair<boost::regex,std::string>(re, str) );
}


// locate_file(+filename,-result)
int
p_locate_file(...)
{
	char* filename;
	if ( EC_succeed != EC_arg(1).is_string ( &filename ) )
	{
		printf( "p_locate_file(): no filename given\n" );
	}
	std::string p = EclipsePath::instance()->locate_file(filename);
	if (EC_succeed != EC_arg(2).unify( EC_word(p.c_str()) ) ){
		printf( "p_locate_file(): could not bind return valie\n" );
		return EC_fail;
	}	
	return EC_succeed;
}


//int main()
//{
//	std::cout << "main" << '\n' ;
//	EclipsePath::create_initial_object();
//	EclipsePath::instance()->add_path_check("@basedir@/src/plugins/readylog");
//	EclipsePath::instance()->add_path("/home/ggierse");
//	std::cout << "created object" << '\n';
//	path res = EclipsePath::instance()->locate_file("fawkes");
//	std::cout << res << '\n';
//	std::cout << "Paths are now: \n";
//	std::vector<std::string> paths = EclipsePath::instance()->paths;
//	for ( std::vector<std::string>::iterator it = paths.begin(); it != paths.end(); ++it){
//		std::cout << *it << '\n';
//	}
//	std::map<boost::regex,std::string> regexes = EclipsePath::instance()->regexes;
//        for ( std::map<boost::regex,std::string>::iterator iit = regexes.begin(); iit != regexes.end(); ++iit){
//                std::cout << iit->first << "=>" << iit->second <<  '\n';
//        }
//}
