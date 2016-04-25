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
#include <utils/system/argparser.h>
#include "plugin_generator.h"

using namespace std;
using namespace fawkes;

/*
 * Usage:
 * Author -> from input
 * Date -> from system
 * Name of the plugin -> from input 
 * Description -> from input
 *
 * Maybe libraries to include in the Makefile and as includes
 */

/*int
main(int argc, char **argv)
{
  ArgumentParser *argparser = new ArgumentParser(argc, argv, "hd:v");
  const vector<const char *> & items = argparser->items();
  if (items.size() == 0 || argparser->has_arg("h")) {
      printf("Help stuff and binary description\n");
  } else {
    printf("Plugin Generator Starting\n");
  }
  */
int
main()
{
  std::string dir = ".";
  std::string author = "Johannes Rothe";
  std::string year = "2016";
  std::string date = "25.04";
  std::string name = "generator_test";
  std::string description = "I am the description";

  PluginGenerator *generator = new PluginGenerator(dir, author, year, date, name, description);

  generator->generate();
}
