/***************************************************************************
 *  gtest_fawkes.cpp - Unit testing in Fawkes
 *    
 *
 *  Created: Aug 24, 2016 11:40:46 PM 2016
 *  Copyright  2016  Frederik Zwilling
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

#include <gtest/gtest.h>
#include <stdexcept>

//for running fawkes in main
#include <baseapp/run.h>
#include <core/exception.h>
#include <cstdio>


//TEST(GTestTest, TestsWorking)
//{
//  ASSERT_EQ(1, 3-2);
//}

/** GTest main function starting fawkes with test plugins
 * test plugins implement tests and call the RUN_ALL_TESTS function
 * @param argc argument count
 * @param argv array of arguments
 */
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  try {
    int retval = 0;

    //init arguments to start fawkes with
    char **fawkes_argv;
    fawkes_argv = new char*[5];
    fawkes_argv[0] = new char[6];
    strcpy(fawkes_argv[0], "fawkes");
    fawkes_argv[1] = new char[2];
    strcpy(fawkes_argv[1], "-p");
    fawkes_argv[2] = new char[128];
    strcpy(fawkes_argv[2], "static-transforms,m-robot-memory,robot_memory_test");
    fawkes_argv[3] = new char[2];
    strcpy(fawkes_argv[3], "-c");
    fawkes_argv[4] = new char[128];
    strcpy(fawkes_argv[4], "gazsim-configurations/default/robotino1.yaml");

    if (! fawkes::runtime::init(5, fawkes_argv, retval)) {
      return retval;
    }
    fawkes::runtime::run();
    fawkes::runtime::cleanup();

    delete [] fawkes_argv;
  } catch (fawkes::Exception &e) {
    printf("Fawkes Test execution ended.\n");
//    e.print_trace();
    return 1;
  }

  return 0;
}
