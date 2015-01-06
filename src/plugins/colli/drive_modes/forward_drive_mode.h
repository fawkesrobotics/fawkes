
/***************************************************************************
 *  forward_drive_mode.h - Implementation of drive-mode "forward"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013-2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_FORWARD_DRIVE_MODE_H_
#define __PLUGINS_COLLI_FORWARD_DRIVE_MODE_H_

#include "abstract_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ForwardDriveModule : public AbstractDriveMode
{
 public:
  ForwardDriveModule(Logger* logger, Configuration* config);
  ~ForwardDriveModule();

  virtual void update();

 private:
  float forward_curvature( float dist_to_target, float dist_to_trajec, float alpha,
                           float cur_trans, float cur_rot );

  float forward_translation( float dist_to_target, float dist_to_front, float alpha,
                             float cur_trans, float cur_rot, float des_rot );
};

} // namespace fawkes

#endif
