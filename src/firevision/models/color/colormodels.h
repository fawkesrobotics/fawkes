
/***************************************************************************
 *  colormodels.h - Inline functions for color models
 *
 *  Generated: Tue May 03 19:50:02 2005
 *  Copyright  2005  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#ifndef __FIREVISION_COLORMODELS_H_
#define __FIREVISION_COLORMODELS_H_

#include <fvutils/base/types.h>

/* To create a new model create a header file that contains an inline function
for the color model.

TODO: We yet have to agree on the arguments and return type to be able to do this
in a general fashion, I will do some measurements to test if this makes sense or
if we can use classes and an interface...
*/

typedef color_t color_classifier_func (unsigned char y, unsigned char u, unsigned char v);

#include "models/color/thresholds.h"

#define classify(y, u, v) classify_by_thresholds((y), (u), (v))

#endif
