/***************************************************************************
 *  getkey.h - getkey returns a keypress in non-blocking manner
 *
 *  Created: Thu Jun 04 19:08:13 2009
 *  Copyright  2009  Masrur Doostdar <doostdar@kbsg.rwth-aachen.de>
 *             2012  Tim Niemueller [www.niemueller.de]
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

#ifndef __UTILS_SYSTEM_GETKEY_H
#define __UTILS_SYSTEM_GETKEY_H

namespace fawkes {
  char getkey(int timeout_decisecs = 0);
} // end namespace fawkes

#endif
