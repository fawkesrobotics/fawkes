
/***************************************************************************
 *  listsrcams.cpp - List available SwissRanger cameras
 *
 *  Created: Wed Jan 13 18:24:39 2010
 *  Copyright  2007-2010  Tim Niemueller [www.niemueller.de]
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

#include <fvutils/system/camargp.h>
#include <fvcams/swissranger.h>

using namespace firevision;

int
main(int argc, char **argv)
{
  SwissRangerCamera::print_available_cams();

  CameraArgumentParser cap("");
  SwissRangerCamera *cam = new SwissRangerCamera(&cap);
  cam->open();
  cam->print_info();
  cam->close();
  delete cam;

  return 0;
}
