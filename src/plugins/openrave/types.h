
/***************************************************************************
 *  types.h - Definition of simple types
 *
 *  Created: Thu Dec 02 13:51:46 2010
 *  Copyright  2010  Bahram Maleki-Fard
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

#ifndef __PLUGINS_OPENRAVE_TYPES_H_
#define __PLUGINS_OPENRAVE_TYPES_H_

#include <utils/math/types.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** Euler rotations. */
typedef enum {
  EULER_ZXZ,		/**< ZXZ rotation */
  EULER_ZYZ,		/**< ZYZ rotation */
  EULER_ZYX		/**< ZYX rotation */
} euler_rotation_t;

} // end namespace firevision

#endif
