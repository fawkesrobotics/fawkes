
/***************************************************************************
 *  eclipse_path.cpp - Eclipse-CLP path externals
 *
 *  Created: Thu Feb 27 15:21:35 2014
 *  Copyright  2014 Gesche Gierse
 *             2014 Tim Niemueller
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

#include "eclipse_path.h"
#include <iterator>
#include <iostream>
#include <core/exception.h>
#include <eclipseclass.h>

using namespace boost::filesystem;
using namespace fawkes;

/** @class EclipsePath
 * Class to determine the location of ECLiPSe-clp programs.
 * Given a filename the complete path to that file will be specified.
 * Paths can contain variables, which will be transformed to the string
 * if matched by a regex.
 * @author Gesche Gierse
 * @author Tim Niemueller
 */


EclipsePath* EclipsePath::m_instance = NULL;


/** Constructor. */
EclipsePath::EclipsePath()
{
}

/** Create the initial EclipsePath object.
 * Already supplies regexes for BASEDIR, CONFDIR and FAWKES_BASEDIR
 */
void
EclipsePath::create_initial_object()
{
  if (m_instance)  return;

  m_instance = new EclipsePath();
  m_instance->add_regex(boost::regex("@BASEDIR@"), BASEDIR);
  m_instance->add_regex(boost::regex("@CONFDIR@"), CONFDIR);
  m_instance->add_regex(boost::regex("@FAWKESDIR@"), FAWKES_BASEDIR);
}

/** Get the EclipsePath instance.
 * @return the instance
 */
EclipsePath* EclipsePath::instance()
{
  create_initial_object();
  return m_instance;
}



/** Add a new path.
 * @param path The path to be added.
 */
void
EclipsePath::add_path(std::string path)
{
  paths.push_back(path);
}


/** Add a new path and apply regexes to all paths.
 * @param path The path to be added.
 */
void
EclipsePath::add_path_check(std::string path)
{
  instance()->add_path(path);
  instance()->apply_regexes();
}

/** Locate a file by filename
 * @param filename the searched filename
 * @return path to the file
 */
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
    try {
      if (exists(p)) {
	//std::cout << "found file " << filename << " at:" << '\n'; 
#ifdef BOOST_FILESYSTEM_VERSION
	return p.native();
#else
	return p.string();
#endif
      }
    }
    catch (const filesystem_error& ex)
    {
      throw Exception( "Filesystem error" );
    }
		
  }
  return "";
}

/** Apply the regexes to all paths.
 */
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

/** Debug method to print all path to the command line.
 */
void
EclipsePath::print_all_paths()
{
  for ( std::vector<std::string>::iterator it = paths.begin(); it != paths.end(); ++it){
    //std::cout << *it << '\n';
  }
}

/** Add a regex. To apply the regex to all paths use
 * apply_regexes().
 * @param re the regex to be matched
 * @param str the string by which each instanstance of the regex will be replaced
 */
void
EclipsePath::add_regex(boost::regex re, std::string str)
{
  regexes.insert( std::pair<boost::regex,std::string>(re, str) );
}

/** Wrapper method for external ECLiPSe-clp.
 * Returns the path to a file.
 * locate_file(+filename,-result)
 */
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
  return p.empty() ? EC_fail : EC_succeed;
}
