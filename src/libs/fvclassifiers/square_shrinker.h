
/***************************************************************************
 *  square_shrinker.h - Header for SquareShrinker
 *
 *  Created: Tue Apr 22 20:53:47 2008 (GO2008, day 4)
 *  Copyright  2005-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __FIREVISION_CLASSIFIER_SQUARE_SHRINKER_H_
#define __FIREVISION_CLASSIFIER_SQUARE_SHRINKER_H_

#include <fvclassifiers/shrinker.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ROI;

class SquareShrinker : public Shrinker
{
 public:

  SquareShrinker();

  virtual void shrink( ROI *roi );
};

} // end namespace firevision

#endif
