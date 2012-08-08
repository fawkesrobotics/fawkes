
/***************************************************************************
 *  qa_yaml.cpp - QA for YAML configuration storage
 *
 *  Created: Wed Aug 01 18:53:22 2012 
 *  Copyright  2012  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

/// @cond QA

#include <config/yaml.h>

#include <iostream>
#include <cstdio>

using namespace std;
using namespace fawkes;

int
main(int argc, char **argv)
{
  YamlConfiguration *config = new YamlConfiguration(CONFDIR);

  try {
    cout << "Loading configuration..." << flush;
    config->load("config.yaml");
    cout << "done" << endl;
  } catch (CouldNotOpenConfigException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }

  unsigned int u = config->get_uint("/fawkes/mainapp/blackboard_size");
  printf("Blackboard size: %u\n", u);

  std::string s = config->get_string("/hardware/roomba/connection_type");
  printf("Roomba connection type: %s\n", s.c_str());

  Configuration::ValueIterator *i = config->get_value("/hardware/roomba/connection_type");
  if (i->next() && i->is_string()) {
    printf("Again as iterator: %s\n", i->get_string().c_str());
  } else {
    printf("!!! Failed, iterator value is not a string\n");
  }
  delete i;

  printf("=== Printing ALL values ===\n");
  i = config->iterator();
  while (i->next()) {
    printf("%s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;


  printf("=== Printing values with prefix /webview ===\n");
  i = config->search("/webview");
  while (i->next()) {
    printf("%s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;

  printf("=== Printing values with prefix /hardware/laser/ ===\n");
  i = config->search("/hardware/laser/");
  while (i->next()) {
    printf("%s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;

  delete config;

  return 0;
}



/// @endcond
