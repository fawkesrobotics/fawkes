/***************************************************************************
 *  defines.h - Defines that are used throughout the colli
 *
 *  Created: Wed Oct 16 18:00:00 2013
 *  Copyright  2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_DEFINES_H_
#define __PLUGINS_COLLI_DEFINES_H_

#define PI 3.14159265

#ifndef _COLLI_CELL_CONSTANTS_
#define _COLLI_CELL_CONSTANTS_     1
#define _COLLI_CELL_OCCUPIED_   1000.0
#define _COLLI_CELL_NEAR_          4.0 // near an obstacle    | COST  6!
#define _COLLI_CELL_MIDDLE_        3.0 // rel.near an obstacle| COST  4!
#define _COLLI_CELL_FAR_           2.0 // far from an obstacle| COST  2!
#define _COLLI_CELL_FREE_          1.0 // default free value  | COST  1!
#endif

#endif
