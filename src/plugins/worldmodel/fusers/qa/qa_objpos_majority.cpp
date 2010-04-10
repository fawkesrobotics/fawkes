
/***************************************************************************
 *  qa_bb_interface.h - BlackBoard interface QA
 *
 *  Generated: Tue Oct 17 15:48:45 2006
 *  Copyright  2006  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/local.h>
#include <blackboard/exceptions.h>
#include <blackboard/bbconfig.h>

#include <interfaces/ObjectPositionInterface.h>

#include <core/exceptions/system.h>
#include <utils/logging/liblogger.h>
#include <utils/logging/logger.h>
#include <utils/logging/console.h>

#include <signal.h>
#include <cstdlib>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// I know this isn't very beautiful :-)
// Is there a library for the worldmodel I could link with?
#include "../fuser.cpp"
#include "../objpos_majority.cpp"

using namespace std;
using namespace fawkes;


bool quit = false;

void
signal_handler(int signum)
{
  quit = true;
}


typedef ObjectPositionInterface Opi;

void
test1(Logger* logger, BlackBoard* bb,
      Opi* own, const vector<Opi*>& foreign, Opi* output)
{
  cout << endl << endl << "Test #1:" << endl;
  WorldModelObjPosMajorityFuser fuser(logger, bb,
                                      "OmniBall", // own_id
                                      "WM Ball *", // foreign_id_pattern
                                      output->id(),
                                      1.0); // self-confidence

  float x = 0.0;
  float y = 0.0;
  int i = 1;
  for (vector<Opi*>::const_iterator it = foreign.begin();
       it != foreign.end(); ++it) {
    Opi* opi = *it;
    opi->set_valid(true);
    opi->set_flags(Opi::FLAG_HAS_WORLD);
    opi->set_visible(true);
    opi->set_world_x(x);
    opi->set_world_y(y);
    opi->set_world_z(0.0);
    opi->write();
    x += 0.1f;
    y += 0.1f;
    if (i == 2) {
      x += 3.0f;
      y += 3.0f;
    } else if (i == 5) {
      x += 5.0f;
      y += 5.0f;
    }
    cout << "   Set foreign " << opi->id() << " world " <<
        "(" << opi->world_x() << ", " << opi->world_y() << ")" << endl;
    ++i;
  }

  own->set_valid(true);
  own->set_flags(Opi::FLAG_HAS_WORLD
                 | Opi::FLAG_HAS_RELATIVE_CARTESIAN
                 | Opi::FLAG_HAS_RELATIVE_POLAR);
  own->set_visible(true);
  own->set_world_x(2.0);
  own->set_world_y(2.0);
  own->set_world_z(0.0);
  own->set_bearing(0.5); // doesn't matter
  own->set_distance(1.5); // set to 0.5 to enforce copy-of-own
  own->set_relative_x(1.7); // set to 0.7 to enforce copy-of-own
  own->set_relative_y(0.7);
  own->write();
  cout << "   Set own " << own->id() << " world " <<
      "(" << own->world_x() << ", " << own->world_y() << ")" << endl;

  fuser.fuse();

  cout << "   Reading output interface.. " << flush;
  output->read();
  cout << "(" << output->world_x() << ", " << output->world_y() << ")";
  cout << endl;

  float expected_x = (3.2f + 3.3f + 3.4f) / 3;
  float expected_y = (3.2f + 3.3f + 3.4f) / 3;
  if (output->world_x() != expected_x ||
      output->world_y() != expected_y) {
    throw Exception("Should have averaged interfaces 2, 3, 4 "\
                    "(starting from 0).");
  }
}

void
test2(Logger* logger, BlackBoard* bb,
      Opi* own, const vector<Opi*>& foreign, Opi* output)
{
  cout << endl << endl << "Test #2:" << endl;
  WorldModelObjPosMajorityFuser fuser(logger, bb,
                                      "OmniBall", // own_id
                                      "WM Ball *", // foreign_id_pattern
                                      output->id(),
                                      1.0); // self-confidence

  float x = 0.0;
  float y = 0.0;
  int i = 1;
  for (vector<Opi*>::const_iterator it = foreign.begin();
       it != foreign.end(); ++it) {
    Opi* opi = *it;
    opi->set_valid(true);
    opi->set_flags(Opi::FLAG_HAS_WORLD);
    opi->set_visible(true);
    opi->set_world_x(x);
    opi->set_world_y(y);
    opi->set_world_z(0.0);
    opi->write();
    x += 0.1f;
    y += 0.1f;
    if (i == 2) {
      x += 3.0f;
      y += 3.0f;
    } else if (i == 4) {
      x += 5.0f;
      y += 5.0f;
    }
    cout << "   Set foreign " << opi->id() << " world " <<
        "(" << opi->world_x() << ", " << opi->world_y() << ")" << endl;
    ++i;
  }

  own->set_valid(true);
  own->set_flags(Opi::FLAG_HAS_WORLD
                 | Opi::FLAG_HAS_RELATIVE_CARTESIAN
                 | Opi::FLAG_HAS_RELATIVE_POLAR
                 );
  own->set_visible(true);
  own->set_world_x(2.0);
  own->set_world_y(2.0);
  own->set_world_z(0.0);
  own->set_bearing(0.5); // doesn't matter
  own->set_distance(1.5); // set to 0.5 to enforce copy-of-own
  own->set_relative_x(1.7); // set to 0.7 to enforce copy-of-own
  own->set_relative_y(0.7);
  own->write();
  cout << "   Set own " << own->id() << " world " <<
      "(" << own->world_x() << ", " << own->world_y() << ")" << endl;

  fuser.fuse();

  cout << "   Reading output interface.. " << flush;
  output->read();
  cout << "(" << output->world_x() << ", " << output->world_y() << ")";
  cout << endl;

  if (output->world_x() != own->world_x() ||
      output->world_y() != own->world_y()) {
    throw Exception("Should have copied own interface because "\
                    "no majority was found.");
  }
}
void
test3(Logger* logger, BlackBoard* bb,
      Opi* own, const vector<Opi*>& foreign, Opi* output)
{
  cout << endl << endl << "Test #3:" << endl;
  WorldModelObjPosMajorityFuser fuser(logger, bb,
                                      "OmniBall", // own_id
                                      "WM Ball *", // foreign_id_pattern
                                      output->id(),
                                      1.0); // self-confidence

  float x = 0.0;
  float y = 0.0;
  int i = 1;
  for (vector<Opi*>::const_iterator it = foreign.begin();
       it != foreign.end(); ++it) {
    Opi* opi = *it;
    opi->set_valid(true);
    opi->set_flags(Opi::FLAG_HAS_WORLD);
    opi->set_visible(true);
    opi->set_world_x(x);
    opi->set_world_y(y);
    opi->set_world_z(0.0);
    opi->write();
    x += 0.1f;
    y += 0.1f;
    if (i == 2) {
      x += 3.0f;
      y += 3.0f;
    } else if (i == 4) {
      x += 5.0f;
      y += 5.0f;
    }
    cout << "   Set foreign " << opi->id() << " world " <<
        "(" << opi->world_x() << ", " << opi->world_y() << ")" << endl;
    ++i;
  }

  own->set_valid(true);
  own->set_flags(Opi::FLAG_HAS_WORLD
                 | Opi::FLAG_HAS_RELATIVE_CARTESIAN
                 | Opi::FLAG_HAS_RELATIVE_POLAR
                 );
  own->set_visible(true);
  own->set_world_x(2.0);
  own->set_world_y(2.0);
  own->set_world_z(0.0);
  own->set_bearing(0.5); // doesn't matter
  own->set_distance(0.5);
  own->set_relative_x(0.7);
  own->set_relative_y(0.7);
  own->write();
  cout << "   Set own " << own->id() << " world " <<
      "(" << own->world_x() << ", " << own->world_y() << ")" << endl;

  fuser.fuse();

  cout << "   Reading output interface.. " << flush;
  output->read();
  cout << "(" << output->world_x() << ", " << output->world_y() << ")";
  cout << endl;

  if (output->world_x() != own->world_x() ||
      output->world_y() != own->world_y()) {
    throw Exception("Should have copied own interface because "\
                    "it's near enough");
  }
}

int
main(int argc, char **argv)
{
  LibLogger::init();
  Logger* logger = new ConsoleLogger();

  signal(SIGINT, signal_handler);

  LocalBlackBoard *lbb = new LocalBlackBoard(BLACKBOARD_MEMSIZE);

  BlackBoard* bb = lbb;


  Opi*         own;
  vector<Opi*> foreign;
  Opi*         output;

  try {
    cout << "Opening interfaces.. " << flush;
    own = bb->open_for_writing<Opi>("OmniBall");
    for (int i = 1; i <= 5; i++) {
      stringstream stream;
      stream << "WM Ball ";
      stream << i;
      string id = stream.str();
      Opi* opi = bb->open_for_writing<Opi>(id.c_str());
      foreign.push_back(opi);
    }
    output = bb->open_for_reading<Opi>("WM Ball");
    cout << "done" << endl;
  } catch (Exception &e) {
    cout << "failed! Aborting" << endl;
    e.print_trace();
    exit(1);
  }

  test1(logger, bb, own, foreign, output);
  test2(logger, bb, own, foreign, output);
  test3(logger, bb, own, foreign, output);

  cout << "Closing interfaces.. " << flush;
  bb->close(own);
  for (vector<Opi*>::iterator it = foreign.begin();
       it != foreign.end(); ++it) {
    Opi* opi = *it;
    bb->close(opi);
  }
  cout << "done" << endl;

  cout << "Deleting blackboard.. " << flush;
  delete bb;
  cout << "done" << endl;
  cout << "Finalizing logger.. " << flush;
  LibLogger::finalize();
  cout << "done" << endl;
}


/// @endcond
