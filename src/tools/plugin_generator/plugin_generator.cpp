 
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
#include <core/exception.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <time.h>
#include <fstream>
#include <sys/stat.h>

using namespace std;


/** @class PluginGenerator "plugin_generator.h
 * Generate basic plugins from minimal input.
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
  _filename_makefile   = "Makefile";

  _plugin_name = plugin_name;
  _plugin_name_underscore = replace_dash_w_undescore(_plugin_name);

  _class_name_thread = format_class_name(_plugin_name_underscore, "Thread");
  _class_name_plugin = format_class_name(_plugin_name_underscore, "Plugin");

  _deflector = "__PLUGINS_" + fawkes::StringConversions::to_upper(_plugin_name_underscore) + "_THREAD_H_";
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
	  "%s%s"
	  " *  Copyright  %s  %s\n"
	  " ****************************************************************************/\n\n"
	  "/*  This program is free software; you can redistribute it and/or modify\n"
	  " *  it under the terms of the GNU General Public License as published by\n"
	  " *  the Free Software Foundation; either version 2 of the License, or\n"
	  " *  (at your option) any later version.\n"
	  " *\n"
	  " *  This program is distributed in the hope that it will be useful,\n"
	  " *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
	  " *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
	  " *  GNU Library General Public License for more details.\n"
	  " *\n"
	  " *  Read the full text in the LICENSE.GPL file in the doc directory.\n"
	  " */\n\n",
	  filename.c_str(), _plugin_name.c_str(),
	  (_creation_date.length() > 0 ) ? " *  Created: " : "",
	  (_creation_date.length() > 0 ) ? _creation_date.c_str() : "",
	  _year.c_str(),  _author.c_str()
	  );
}

/** Write makefile header.
 * @param f file to write to
 */
void
PluginGenerator::write_makefile_header(FILE *f)
{
  fprintf(f,
          "#*****************************************************************************\n"
          "#         Makefile Build System for Fawkes: %s Plugin\n"
          "#                            -------------------\n"
          "#   Created on %s \n"
          "#   Copyright (C) %s by %s\n"
          "#\n"
          "#*****************************************************************************\n"
          "#\n"
          "#   This program is free software; you can redistribute it and/or modify\n"
          "#   it under the terms of the GNU General Public License as published by\n"
          "#   the Free Software Foundation; either version 2 of the License, or\n"
          "#   (at your option) any later version.\n"
          "#\n"
          "#*****************************************************************************\n\n",
          _plugin_name.c_str(), _creation_date.c_str(), _year.c_str(),
          _author.c_str());
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
	  "using namespace fawkes;\n\n"
	  "/** @class %s '%s' \n"
	  " * %s\n"
	  " * @author %s\n"
	  " */\n\n",
	  _filename_thread_h.c_str(),
	  _class_name_thread.c_str(), _filename_thread_h.c_str(), _description.c_str(),
	  _author.c_str());
  //Constructor
  fprintf(f,
          "/** Constructor. */\n"
          "%s::%s()\n"
          " : Thread(\"%s\", Thread::OPMODE_WAITFORWAKEUP),\n"
          "   BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_ACT) \n{\n}\n\n",
          //TODO support the other OPMODES
          _class_name_thread.c_str(), _class_name_thread.c_str(),
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
          "#include <core/threading/thread.h>\n"
          "#include <aspect/blocked_timing.h>\n"
          "#include <aspect/logging.h>\n"
          "#include <aspect/blackboard.h>\n"
          "#include <aspect/configurable.h>\n\n"

          "namespace fawkes {\n"
          "  // add forward declarations here, e.g., interfaces\n"
          "}\n\n"
	  "class %s \n"
          ": public fawkes::Thread,\n"
          "  public fawkes::BlockedTimingAspect,\n"
          "  public fawkes::LoggingAspect,\n"
          "  public fawkes::ConfigurableAspect,\n"
          "  public fawkes::BlackBoardAspect\n"
	  "{\n\n"
	  " public:\n"
          "  %s();\n\n"
          "  virtual void init();\n"
          "  virtual void finalize();\n"
          "  virtual void loop();\n\n"
          "  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */\n"
          "  protected: virtual void run() { Thread::run(); }\n\n"
          " private:\n"
          "  //Define class member variables here\n"
          ,
	  _class_name_thread.c_str(),
	  _class_name_thread.c_str());

  fprintf(f, "\n};\n\n\n#endif");
}

/** Write plugin cpp file.
 * @param f file to write to
 */
void
PluginGenerator::write_plugin_cpp(FILE *f)
{
  write_header(f, _filename_plugin_cpp);
  fprintf(f,
          "#include <core/plugin.h>\n\n"
          "#include \"%s\"\n\n"
          "using namespace fawkes;\n\n",
          _filename_thread_h.c_str());
  fprintf(f,
          "/** @class %s \"%s\"\n"
          " * %s\n"
          " * @author %s\n"
          " */\n",
          _class_name_plugin.c_str(), _filename_plugin_cpp.c_str(),
          _description.c_str(), _author.c_str());
  fprintf(f,
          "class %s : public fawkes::Plugin\n"
          "{\n"
          " public:\n"
          "  /** Constructor.\n"
          "   * @param config Fakwes configuration\n"
          "   */\n"
          "  %s(Configuration *config)\n"
          "     : Plugin(config)\n"
          "  {\n"
          "     thread_list.push_back(new %s());\n"
          "  }\n"
          "};\n\n",
          _class_name_plugin.c_str(), _class_name_plugin.c_str(),
          _class_name_thread.c_str());
  fprintf(f,
          "PLUGIN_DESCRIPTION(\"%s\")\n"
          "EXPORT_PLUGIN(%s)",
          _description.c_str(), _class_name_plugin.c_str());
}

/** Write Makefile.
 * @param f file to write to
 */
void
PluginGenerator::write_makefile (FILE* f)
{
  write_makefile_header(f);
  std::string filename_plugin_o = _plugin_name + "_plugin.o";
  std::string filename_thread_o = _plugin_name + "_thread.o";
  fprintf(f,
          "BASEDIR = ../../..\n"
          "include $(BASEDIR)/etc/buildsys/config.mk\n\n"
          "LIBS_%s = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \\\n"
          "                      fawkesblackboard fawkesinterface\n\n"
          "OBJS_%s = %s %s\n\n",
          _plugin_name_underscore.c_str(), _plugin_name_underscore.c_str(), filename_plugin_o.c_str(),
          filename_thread_o.c_str()
          );
  fprintf(f,
         "PLUGINS_all = $(PLUGINDIR)/%s.$(SOEXT)\n\n"
         "OBJS_all = $(OBJS_%s)\n\n"
         "include $(BUILDSYSDIR)/base.mk",
         _plugin_name.c_str(), _plugin_name.c_str());
}

/** Replace dash with underscore.
 * Example: plugin-generator to plugin_generator
 * @param source input string
 * @return modified string
 */
std::string
PluginGenerator::replace_dash_w_undescore(std::string source)
{
  for(std::string::size_type i = 0; (i = source.find("-", i)) != std::string::npos;)
  {
    source.replace(i, 1, "_");
    i++;
  }
  return source;
}

/** Format a lowercase plugin name to CamelCase class.
 * Example: plugin_name to PluginNameThread
 * @param plugin_name name of plugin
 * @param appendix class name appendix, e.g., Thread or Plugin
 * @return class name matching the plugin name
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
  FILE *makefile;

  struct stat info;

  if (!(stat(_dir.c_str(), &info) == 0 && S_ISDIR(info.st_mode))) {
	  if (mkdir(_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1) {
		  throw fawkes::Exception(errno, "Failed to generate plugin, cannot create directory");
	  }
  }
  thread_h   = fopen(string(_dir + _filename_thread_h).c_str(), "w");
  thread_cpp = fopen(string(_dir + _filename_thread_cpp).c_str(), "w");
  plugin_cpp = fopen(string(_dir + _filename_plugin_cpp).c_str(), "w");
  makefile   = fopen(string(_dir + _filename_makefile).c_str(), "w");

  if ( thread_h == NULL ) {
    printf("Cannot open thread_h file %s%s\n", _dir.c_str(), _filename_thread_h.c_str());
  }
  if ( thread_cpp == NULL ) {
    printf("Cannot open thread_cpp file %s%s\n", _dir.c_str(), _filename_thread_cpp.c_str());
  }
  if ( plugin_cpp == NULL ) {
    printf("Cannot open plugin_cpp file %s%s\n", _dir.c_str(), _filename_plugin_cpp.c_str());
  }
  if ( makefile == NULL ) {
    printf("Cannot open makefile %s%s\n", _dir.c_str(), _filename_makefile.c_str());
  }
  
  write_thread_cpp(thread_cpp);
  write_thread_h(thread_h);
  write_plugin_cpp(plugin_cpp);
  write_makefile(makefile);

  fclose(thread_cpp);
  fclose(thread_h);
  fclose(plugin_cpp);
  fclose(makefile);

  printf("Plugin %s successfully created!\n",  _plugin_name.c_str());
}
