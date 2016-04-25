 
/***************************************************************************
 *  cpp_generator.cpp - C++ Interface generator
 *
 *  Created: Thu Oct 12 02:01:27 2006
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

#include "plugin_generator.h"

#include <utils/misc/string_conversions.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <time.h>
#include <fstream>

using namespace std;


/** @class CppInterfaceGenerator <interfaces/generator/cpp_generator.h>
 * Generator that transforms input from the InterfaceParser into valid
 * C++ classes.
 */

/** Constructor.
 * @param directory Directory where to create the files
 * @param author Author of the plugin
 * @param year Year of copyright
 * @param creation_date Creation date of the plugin
 * @param plugin_name Name of the plugin
 * @param description Plugin description
 */
PluginGenerator::PluginGenerator(std::string directory,
					     std::string author,
					     std::string year, std::string creation_date,
					     std::string plugin_name, std::string description
					     )
{
  _dir    = directory;
  if ( _dir.find_last_of("/") != (_dir.length() - 1) ) {
    _dir += "/";
  }
  _author = author;
  _year   = year;
  _creation_date = creation_date;
  _description = description;

  _filename_thread_cpp = plugin_name + "_thread.cpp";
  _filename_thread_h   = plugin_name + "_thread.h";
  _filename_plugin_cpp = plugin_name + "_plugin.cpp";

  _plugin_name = plugin_name;

  _class_name_thread = format_class_name(_plugin_name, "Thread");
  _class_name_plugin = format_class_name(_plugin_name, "Plugin");

  _deflector = "__PLUGINS_" + fawkes::StringConversions::to_upper(plugin_name) + "THREAD_H_";
}


/** Destructor */
PluginGenerator::~PluginGenerator()
{
}

/** Write header to file.
 * @param f file to write to
 * @param filename name of file
 */
void
PluginGenerator::write_header(FILE *f, std::string filename)
{
  fprintf(f,
	  "\n/***************************************************************************\n"
	  " *  %s - %s\n"
	  " *\n"
	  "%s%s%s"
	  " *  Copyright  %s  %s\n"
	  " *\n"
	  " ****************************************************************************/\n\n"
	  "/*  This program is free software; you can redistribute it and/or modify\n"
	  " *  it under the terms of the GNU General Public License as published by\n"
	  " *  the Free Software Foundation; either version 2 of the License, or\n"
	  " *  (at your option) any later version. A runtime exception applies to\n"
	  " *  this software (see LICENSE.GPL_WRE file mentioned below for details).\n"
	  " *\n"
	  " *  This program is distributed in the hope that it will be useful,\n"
	  " *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
	  " *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
	  " *  GNU Library General Public License for more details.\n"
	  " *\n"
	  " *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.\n"
	  " */\n\n",
	  filename.c_str(), _plugin_name.c_str(),
	  (_creation_date.length() > 0 ) ? " *  Interface created: " : "",
	  (_creation_date.length() > 0 ) ? _creation_date.c_str() : "",
	  (_creation_date.length() > 0 ) ? "\n" : "",
	  _year.c_str(), (_author.length() > 0) ? _author.c_str() : "AllemaniACs RoboCup Team"
	  );
}


/** Write header deflector.
 * @param f file to write to
 */
void
PluginGenerator::write_deflector(FILE *f)
{
  fprintf(f, "#ifndef %s\n", _deflector.c_str());
  fprintf(f, "#define %s\n\n", _deflector.c_str());
}


/** Write cpp file.
 * @param f file to write to
 */
void
PluginGenerator::write_thread_cpp(FILE *f)
{
  write_header(f, _filename_thread_cpp);
  fprintf(f,
          "#include \"%s\"\n\n"
	  "using namespace fawkes \n\n"
	  "/** @class %s '%s' \n"
	  " * %s\n"
	  " * @author %s\n"
	  " */\n\n",
	  _filename_thread_h.c_str(),
	  _class_name_thread.c_str(), _filename_thread_h.c_str(), _description.c_str(),
	  _author.c_str());
  //Constructor
  fprintf(f,
          "%s::%s()\n"
          " : Thread(\"%s\", Thread::OPMODE_CONTINUOUS)\n{\n}\n\n", //TODO support the other OPMODES
          _class_name_thread.c_str(), _class_name_thread.c_str(),
          _class_name_thread.c_str());
  //Destructor
  fprintf(f,
          "%s::~%s()\n{\n}\n\n",
          _class_name_thread.c_str(),
          _class_name_thread.c_str());
  //init
  fprintf(f,
          "void\n%s::init()\n{\n}\n\n", _class_name_thread.c_str());
  //loop
  fprintf(f,
          "void\n%s::loop()\n{\n}\n\n", _class_name_thread.c_str());
  //finalize
  fprintf(f,
          "void\n%s::finalize()\n{\n}\n\n", _class_name_thread.c_str());
}

/** Write h file.
 * @param f file to write to
 */
void
PluginGenerator::write_thread_h(FILE *f)
{
  write_header(f, _filename_thread_h);
  write_deflector(f);

  fprintf(f,
          "#include <string>\n"
          "#include <core/threading/thread.h>\n"
          "#include <aspect/blocked_timing.h>\n"
          "#include <aspect/logging.h>\n"
          "#include <aspect/configurable.h>\n\n"

	  "namespace fawkes {\n"
          "}\n\n"
	  "class %s \n"
          ": public fawkes::Thread,\n"
          "  public fawkes::BlockedTimingAspect,\n"
          "  public fawkes::LoggingAspect,\n"
          "  public fawkes::ConfigurableAspect,\n"
          "  public fawkes::BlackBoardAspect\n"
	  "{\n\n"
	  " public:\n"
          "  %s()\n\n"
          "  virtual void init()\n"
          "  virtual void finalize()\n"
          "  virtual void loop()\n\n"
          "  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */\n"
          "  protected: virtual void run() { Thread::run(); }\n\n"
          " private:\n"
          ,
	  _class_name_thread.c_str(),
	  _class_name_thread.c_str());

  fprintf(f, "\n};\n\n\n#endif");
}

/*
 * Format a lowercase plugin name to CamelCase plugin name and append
 * a string to specify the name
 *
 * Example: plugin_name to PluginNameThread
 */
std::string
PluginGenerator::format_class_name(std::string plugin_name, std::string appendix)
{
  std::string class_name;
  //check if there is an underline in the plugin name
  std::size_t underline_position = plugin_name.find('_');
  if (underline_position!=std::string::npos){
   //Eliminate underscores
   std::istringstream stream(plugin_name);
   std::string item;
   std::vector<std::string> splitted;
   while (std::getline(stream, item, '_')) {
       splitted.push_back(item);
   }
   //camelcase the words
   for (auto element:splitted){
     element[0] = std::toupper(element[0]);
     class_name.append(element);
   }
   class_name.append(appendix);
  } else {
    //Use the name and append
    plugin_name[0] = std::toupper(plugin_name[0]);
    class_name.append(plugin_name);
    class_name.append(appendix);
  }
  return class_name;
}


/** Generator cpp and h files.
 */
void
PluginGenerator::generate()
{
  FILE *thread_cpp;
  FILE *thread_h;
  FILE *plugin_cpp;
  //TODO Makefile, Pluginfile

  thread_h   = fopen(string(_dir + _filename_thread_h).c_str(), "w");
  thread_cpp = fopen(string(_dir + _filename_thread_cpp).c_str(), "w");
  plugin_cpp = fopen(string(_dir + _filename_plugin_cpp).c_str(), "w");

  if ( thread_h == NULL ) {
    printf("Cannot open thread_h file %s%s\n", _dir.c_str(), _filename_thread_h.c_str());
  }
  if ( thread_cpp == NULL ) {
    printf("Cannot open thread_cpp file %s%s\n", _dir.c_str(), _filename_thread_cpp.c_str());
  }
  if ( plugin_cpp == NULL ) {
    printf("Cannot open plugin_cpp file %s%s\n", _dir.c_str(), _filename_plugin_cpp.c_str());
  }
  
  write_thread_cpp(thread_cpp);
  write_thread_h(thread_h);

  fclose(thread_cpp);
  fclose(thread_h);
}
