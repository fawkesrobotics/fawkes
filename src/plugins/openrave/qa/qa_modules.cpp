
/***************************************************************************
 *  qa_modules.cpp - QA for OpenRAVE Environment class
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

#include <openrave/openrave.h>

#include <plugins/openrave/environment.h>
#include <plugins/openrave/robot.h>
#include <logging/console.h>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <vector>
#include <list>

using namespace fawkes;
using namespace std;

void
printVector(vector<float> &v)
{
  stringstream s;
  //printf("## size:%u \n", v.size());
  for(unsigned int i=0; i<v.size(); i++)
  {
    s << "(" << i << ")" << v[i] << "    ";
    //printf("## %u:)%f \n", i, v[i]);
  }
  printf("%s \n", s.str().c_str());
}

int
main(int argc, char **argv)
{
  printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

  string robotFile = SRCDIR"/../manipulators/katana.robot.xml";

  ConsoleLogger* cl = new ConsoleLogger();

  OpenRaveRobotPtr robot( new OpenRaveRobot(cl) );
  OpenRaveEnvironmentPtr env( new OpenRaveEnvironment(cl) );

  vector<OpenRAVE::RobotBasePtr> robots;
  list<OpenRAVE::ModuleBasePtr> modules;



  env->create();

  env->get_env_ptr()->GetModules(modules);
  env->get_env_ptr()->GetRobots(robots);
  cl->log_debug("qa_modules", "Environment created");
  cl->log_debug("qa_modules", "#modules:%u  #robots:%u", modules.size(), robots.size());




  try {
    robot->load(robotFile, env);
  } catch (Exception &e) {
    cl->log_error("qa_modules", "error:%s", e.what());
    return 0;
  }

  env->get_env_ptr()->GetModules(modules);
  env->get_env_ptr()->GetRobots(robots);
  cl->log_debug("qa_modules", "Robot loaded");
  cl->log_debug("qa_modules", "#modules:%u  #robots:%u", modules.size(), robots.size());



  env->add_robot(robot);
  robot->set_ready();

  env->get_env_ptr()->GetModules(modules);
  env->get_env_ptr()->GetRobots(robots);
  cl->log_debug("qa_modules", "Robot initialized");
  cl->log_debug("qa_modules", "#modules:%u  #robots:%u", modules.size(), robots.size());



  robot = NULL;

  env->get_env_ptr()->GetModules(modules);
  env->get_env_ptr()->GetRobots(robots);
  cl->log_debug("qa_modules", "Robot Destroyed");
  cl->log_debug("qa_modules", "#modules:%u  #robots:%u", modules.size(), robots.size());




  env->destroy();

  return 0;
}


/// @endcond
