
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
    config->load("config.yaml", "");
    cout << "done" << endl;
  } catch (CouldNotOpenConfigException &e) {
    cout << "failed" << endl;
    e.print_trace();
  }


  delete config;

  return 0;
}



/// @endcond
