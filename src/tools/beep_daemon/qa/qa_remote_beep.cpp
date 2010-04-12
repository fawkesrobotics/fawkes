
/***************************************************************************
 *  qa_beep.cpp - Order beep via remote b
 *
 *  Created: Sun Apr 11 22:21:58 2010
 *  Copyright  2006-2010  Tim Niemueller [www.niemueller.de]
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

#include <blackboard/remote.h>
#include <interfaces/SwitchInterface.h>
#include <utils/system/argparser.h>

#include <cstdio>

using namespace fawkes;

int
main(int argc, char **argv)
{
  ArgumentParser argp(argc, argv, "01f:d:");

  BlackBoard *rbb = new RemoteBlackBoard("localhost", 1910);
  SwitchInterface *si = rbb->open_for_reading<SwitchInterface>("Beep");

  if (argp.has_arg("1")) {
    si->msgq_enqueue(new SwitchInterface::EnableSwitchMessage());
  } else if (argp.has_arg("0")) {
    si->msgq_enqueue(new SwitchInterface::DisableSwitchMessage());
  } else if (argp.has_arg("d")) {
    if ( ! argp.has_arg("f") ) {
      printf("Argument -d requires to have -f as well\n");
    } else {
      float d = argp.parse_float("d");
      float f = argp.parse_float("f");
      si->msgq_enqueue(new SwitchInterface::EnableDurationMessage(d, f));
    }
  } else if (argp.has_arg("f")) {
    float f = argp.parse_float("f");
    si->msgq_enqueue(new SwitchInterface::SetMessage(true, f));
  }

  rbb->close(si);
  delete rbb;
}


/// @endcond
