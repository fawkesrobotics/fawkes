
/***************************************************************************
 *  eclipse_path.h - Eclipse-CLP path externals
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

#ifndef __PLUGINS_ECLIPSE_CLP_EXTERNALS_ECLIPSE_PATH_H_
#define __PLUGINS_ECLIPSE_CLP_EXTERNALS_ECLIPSE_PATH_H_

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
  
 public: /* members */
  std::vector<std::string>            paths;    //!< all paths known
  std::map<boost::regex, std::string> regexes;  /**< regexes and strings they should be replaced with */
};

extern "C" int p_locate_file(...);

#endif
