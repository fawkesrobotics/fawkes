
/***************************************************************************
 *  bb_cleanup.cpp - cleanup unused Fawkes BlackBoard shared memory segments
 *
 *  Generated: Thu Oct 19 14:56:13 2006 (Anne's 25th Birthday)
 *  Copyright  2005-2006  Tim Niemueller [www.niemueller.de]
 *
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

#include <blackboard/local.h>
#include <config/sqlite.h>
#include <iostream>

using namespace std;
using namespace fawkes;

int
main(int argc, char **argv)
{
  SQLiteConfiguration config(CONFDIR);
  config.load();

  std::string token = "";
  try {
    token = config.get_string("/fawkes/mainapp/blackboard_magic_token");
  } catch (Exception &e) {
    cout << "Could not read shared memory token for blackboard." << endl;
    cout << "BlackBoard is probably running without shared memory." << endl;
    return -1;
  }

  LocalBlackBoard::cleanup(token.c_str());
  return 0;
}
