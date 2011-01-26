
/***************************************************************************
 *  listfwcams.cpp - List available firewire cameras
 *
 *  Created: Tue Jul 03 11:23:25 2007 (RoboCup 2007, Atlanta)
 *  Copyright  2007  Tim Niemueller [www.niemueller.de]
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

#include <fvcams/firewire.h>

using namespace firevision;

int
main(int argc, char **argv)
{
  FirewireCamera::print_available_fwcams();
  return 0;
}
