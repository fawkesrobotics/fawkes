 
/***************************************************************************
 *  cpp_generator.h - C++ Interface generator
 *
 *  Created: Thu Oct 12 01:59:02 2006
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
 *
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

#ifndef __PLUGINS_GENERATOR_H_
#define __PLUGINS_GENERATOR_H_

#include <vector>
#include <string>
#include <sstream>
#include <stdio.h>

class PluginGenerator
{
 public:
  PluginGenerator(std::string directory,
			std::string author,
			std::string year, std::string creation_date,
			std::string plugin_name, std::string description
			);
  ~PluginGenerator();

  void write_thread_h(FILE *f);
  void write_thread_cpp(FILE *f);
  void write_plugin_cpp(FILE *f);
  void write_makefile(FILE *f);
  void write_makefile_header(FILE *f);
  void write_header(FILE *f, std::string filename);
  void write_deflector(FILE *f);
  std::string format_class_name(std::string plugin_name, std::string append);
  std::string replace_dash_w_undescore(std::string source);
  void generate();

 private:
  std::string _dir;
  std::string _plugin_name;
  std::string _plugin_name_underscore;
  std::string _class_name_thread;
  std::string _class_name_plugin;
  std::string _description;
  std::string _filename_thread_cpp;
  std::string _filename_thread_h;
  std::string _filename_plugin_cpp;
  std::string _filename_makefile;
  std::string _deflector;
  std::string _author;
  std::string _year;
  std::string _creation_date;
};


#endif
