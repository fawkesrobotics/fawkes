
/***************************************************************************
 *  ffinfo.cpp - Fawkes info tool
 *
 *  Created: Fri Aug 07 23:37:57 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
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

#include <core/version.h>

#include <cstdio>


/** Info tool main.
 * @param argc argument count
 * @param argv arguments
 */
int
main(int argc, char **argv)
{
  printf("Fawkes %u.%u.%u\n",
	 FAWKES_VERSION_MAJOR, FAWKES_VERSION_MINOR, FAWKES_VERSION_MICRO);

  return 0;
}
