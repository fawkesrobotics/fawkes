
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

EclipsePath* EclipsePath::m_instance = NULL;


/** Constructor. */
EclipsePath::EclipsePath()
{
}

void
EclipsePath::create_initial_object()
{
  if (m_instance)  return;

  m_instance = new EclipsePath();
  m_instance->add_regex(boost::regex("@BASEDIR@"), BASEDIR);
  m_instance->add_regex(boost::regex("@CONFDIR@"), CONFDIR);
  m_instance->add_regex(boost::regex("@FAWKESDIR@"), FAWKES_BASEDIR);
}

EclipsePath* EclipsePath::instance()
{
  create_initial_object();
  return m_instance;
}



void
EclipsePath::add_path(std::string path)
{
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
  return p.empty() ? EC_fail : EC_succeed;
}
