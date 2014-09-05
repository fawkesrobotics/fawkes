
/***************************************************************************
 *  qa_robot.cpp - QA for OpenRAVE Environment class
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


#include <plugins/openrave/types.h>
#include <plugins/openrave/environment.h>
#include <plugins/openrave/robot.h>
#include <plugins/openrave/manipulators/katana6M180.h>
#include <logging/console.h>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <vector>

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

  OpenRaveManipulatorPtr manip( new OpenRaveManipulatorKatana6M180(6, 5) );
  OpenRaveRobotPtr robot( new OpenRaveRobot(cl) );
  OpenRaveEnvironmentPtr env( new OpenRaveEnvironment(cl) );

  env->create();

  try {
    robot->load(robotFile, env);
  } catch (Exception &e) {
    cl->log_error("qa_robot", "error:%s", e.what());
    return 0;
  }

  // configure manip
  manip->add_motor(0,0);
  manip->add_motor(1,1);
  manip->add_motor(2,2);
  manip->add_motor(4,3);
  manip->add_motor(5,4);
  robot->set_manipulator(manip);

  env->add_robot(robot);
  robot->set_ready();
  env->lock();


  vector<float> val, v;
  val.push_back(0.1);
  val.push_back(0.2);
  val.push_back(0.3);
  val.push_back(0.4);
  val.push_back(0.5);

  manip->set_angles_device(val);
  manip->get_angles(v);
  printVector(v);
  manip->get_angles_device(v);
  printVector(v);


  env->start_viewer();

  //print angles taken from OpenRAVE Model (can be modified in GUI)
  while(1) {
    robot->update_manipulator();
    manip->get_angles(v);
    printVector(v);
    manip->get_angles_device(v);
    printVector(v);
    usleep(1000*500);
  }


  env->destroy();

  return 0;
}


/// @endcond
