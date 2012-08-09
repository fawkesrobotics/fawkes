
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
  printf("=== Loading configuration ===\n");
    config->load("config.yaml");
    cout << "...done" << endl;
  } catch (CouldNotOpenConfigException &e) {
    cout << "...failed" << endl;
    e.print_trace();
    return -1;
  }


  printf("\n\n=== Reading some assorted values ===\n");

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

  printf("\n\n=== Printing ALL values ===\n");
  i = config->iterator();
  while (i->next()) {
    printf("%s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;


  printf("\n\n=== Printing values with prefix /webview ===\n");
  i = config->search("/webview");
  while (i->next()) {
    printf("%s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;

  printf("\n\n=== Printing values with prefix /hardware/laser/ ===\n");
  i = config->search("/hardware/laser/");
  while (i->next()) {
    printf("%s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;

  printf("\n\n=== Setting /z/foo/bar to test ===\n");
  config->set_string("/z/foo/bar", "test");
  printf("Reading back: %s\n", config->get_string("/z/foo/bar").c_str());


  printf("\n\n=== Erase test ===\n");
  config->set_string("/z/erase/1", "test1");
  config->set_string("/z/erase/2", "test2");
  config->set_string("/z/erase/3", "test3");
  config->set_string("/z/erase/4", "test4");
  config->set_string("/z/erase/5", "test5");
  printf("- Before erasing:\n");
  i = config->search("/z/erase");
  while (i->next()) {
    printf("  %s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;

  printf("- Now erasing /z/erase/4... afterwards:\n");
  config->erase("/z/erase/4");
  i = config->search("/z/erase");
  while (i->next()) {
    printf("  %s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;

  printf("- Now erasing /z/erase/6 (which does not exist)\n");
  try {
    config->erase("/z/erase/6");
  } catch (Exception &e) {
    printf("  Got exception as expected: %s\n", e.what_no_backtrace());
  }


  config->set_string("/z/erase/second/1", "test1");
  config->set_string("/z/erase/second/2", "test2");
  printf("- Before second erasing:\n");
  i = config->search("/z/erase");
  while (i->next()) {
    printf("  %s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;

  printf("- Now erasing /z/erase/second/*... afterwards:\n");
  config->erase("/z/erase/second/1");
  config->erase("/z/erase/second/2");
  i = config->search("/z/erase");
  while (i->next()) {
    printf("  %s: %s (%s)\n", i->path(), i->get_as_string().c_str(), i->type());
  }
  delete i;


  delete config;

  return 0;
}



/// @endcond
