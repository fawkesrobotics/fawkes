
/***************************************************************************
 *  qa_env.cpp - QA for OpenRAVE Environment class
 *
 *  Created: Thu Sep 16 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

// Do not include in api reference
///@cond QA

//#include <openrave-core.h>
#include <plugins/openrave/environment.h>
#include <logging/console.h>
#include <cstdio>
#include <iostream>
#include <vector>

using namespace fawkes;
using namespace std;

void
printVector(std::vector<float> &v)
{
  printf("## size:%lu \n", v.size());
  for(unsigned int i=0; i<v.size(); i++)
  {
    printf("## %u:)%f \n", i, v[i]);
  }
}

int
main(int argc, char **argv)
{
  printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

  ConsoleLogger* cl = new ConsoleLogger();

  //OpenRAVE::RaveInitialize(true); //optional..should be done automatically if not explicitly implemented

  OpenRaveEnvironment* env;
  env = new OpenRaveEnvironment(cl);
  env->create();

  env->enable_debug();

  string robotFile = SRCDIR"/../manipulators/katana.robot.xml";
  env->add_robot(robotFile);

  env->lock();

  env->start_viewer();

  usleep(1000*10000);

  env->destroy();

  return 0;
}


/// @endcond
