/***************************************************************************
 *  main.cpp - Interface generator main app
 *
 *  Generated: Tue Oct 10 17:42:05 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <string>
#include <cstdio>
#include <ctime>
#include <utils/system/argparser.h>
#include "plugin_generator.h"

using namespace std;
using namespace fawkes;

void
print_usage(const char *program_name)
{
  printf("Usage: %s [-h] <author_name> <plugin_name> <description> <directory> \n"
	 "Example: %s \"John Doe\" robot_mover \"Move the robot a meter forward\" \n"
	 "                            ~/fawkes/src/plugins/robot_mover/"
	 "\n"
         "-h  Print this usage information\n\n"
         "Generate the necessary files to build a fawkes plugin\n",
	 program_name, program_name);
}

void
generate_plugin(std::string author_name, std::string plugin_name, std::string description, std::string directory)
{
  time_t now = time(0);
  std::string date = ctime(&now);
  tm *time_structure = localtime(&now);
  std::string year = std::to_string(time_structure->tm_year + 1900);
  //TimeStructure's year is the number of years since 1900

  PluginGenerator *generator = new PluginGenerator(directory, author_name, year , date, plugin_name, description);
  generator->generate();
}

bool
plugin_name_valid(std::string plugin_name){
  for (char& c : plugin_name){
    if (isalpha(c) || c == '-' || c == '_') {
    } else {
      return false;
    }
  }
  return true;
}

int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "h");

  if ( argp.has_arg("h") ) {
    print_usage(argv[0]);
    exit(0);
  }

  std::string author_name, plugin_name, description, directory;
  if (argp.num_items() != 4) {
    printf("ERROR: Invalid number of arguments\n");
    print_usage(argv[0]);
    exit(1);
  } else if (!plugin_name_valid(argp.items()[1])) {
    printf("ERROR: Invalid plugin name: Only alphanumerical chars allowed. \n"
        "To separate multiple words use '-' or '_'\n");
    exit(2);
  }
    else {
    author_name = argp.items()[0];
    plugin_name = argp.items()[1];
    description = argp.items()[2];
    directory = argp.items()[3];
    generate_plugin(author_name, plugin_name, description, directory);
  }

  return 0;
}
