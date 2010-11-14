
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

#include <openrave/connector.h>
#include <openrave/manipulators/katana6M180.h>
#include <utils/logging/console.h>
#include <cstdio>
#include <iostream>
#include <vector>

using namespace fawkes;
using namespace std;

void
printVector(vector<float> v)
{
  printf("## size:%u \n", v.size());
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
  string robotFile = "../src/plugins/openrave/xml/caesar.robot.xml";

  OpenRAVEConnector* con = new OpenRAVEConnector(cl);
  con->setup(robotFile);

 // configure manipulator
  OpenRAVEManipulatorKatana6M180* __manipKatana = new OpenRAVEManipulatorKatana6M180(6, 5);
  __manipKatana->addMotor(0,0);
  __manipKatana->addMotor(1,1);
  __manipKatana->addMotor(2,2);
  __manipKatana->addMotor(4,3);
  __manipKatana->addMotor(5,4);

  con->setManipulator(__manipKatana);

  //con->startViewer();


  //usleep(5000*1000);
  //free(con);





  return 1;
}


/// @endcond
