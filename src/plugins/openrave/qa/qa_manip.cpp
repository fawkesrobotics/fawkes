
/***************************************************************************
 *  qa_manip.cpp - QA for OpenRAVE Manipulator class
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

#include <openrave/manipulators/katana6M180.h>
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

  OpenRAVEManipulator m(6, 5);
  OpenRAVEManipulatorKatana6M180 k(6, 5);
  vector<float> v;
  vector<float> val;

  m.addMotor(0,0);
  m.addMotor(1,1);
  m.addMotor(2,2);
  m.addMotor(4,3);
  m.addMotor(5,4);

  k.addMotor(0,0);
  k.addMotor(1,1);
  k.addMotor(2,2);
  k.addMotor(4,3);
  k.addMotor(5,4);

  val.push_back(0.1);
  val.push_back(0.2);
  val.push_back(0.3);
  val.push_back(0.4);
  val.push_back(0.5);

  //set angles
  m.setAnglesDevice(val);
  k.setAnglesDevice(val);

  //print angles
  m.getAngles(v);
  printVector(v);

  m.getAnglesDevice(v);
  printVector(v);

  k.getAngles(v);
  printVector(v);

  k.getAnglesDevice(v);
  printVector(v);


  // test manipulator pointer. same as above, set angles, then print
  OpenRAVEManipulator* p;

  p = new OpenRAVEManipulatorKatana6M180(6, 5);
  p->addMotor(0,0);
  p->addMotor(1,1);
  p->addMotor(2,2);
  p->addMotor(4,3);
  p->addMotor(5,4);

  p->setAnglesDevice(val);

  p->getAngles(v);
  printVector(v);

  p->getAnglesDevice(v);
  printVector(v);


  return 0;
}


/// @endcond
