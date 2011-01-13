
/***************************************************************************
 *  calibration.h - Abstract class defining a camera calibration matrix K
 *                  for a finite camera
 *
 *  Created: Thu May 08 13:24:00 2008
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
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

#ifndef __FIREVISION_CALIBRATION_H__
#define __FIREVISION_CALIBRATION_H__

#include <geometry/matrix.h>

namespace firevision {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class Calibration: public fawkes::Matrix
{
 public:
  Calibration(const Calibration& cal);
  Calibration(const fawkes::Matrix& k);
  virtual ~Calibration();

  Matrix K() const;

 protected:
  Calibration();

  Calibration& K(const fawkes::Matrix& k);
};

} // end namespace firevision

#endif // __FIREVISION_CALIBRATION_H__
