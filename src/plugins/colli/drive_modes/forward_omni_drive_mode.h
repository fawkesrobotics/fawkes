
/***************************************************************************
 *  forward_omni_drive_mode.h - Implementation of drive-mode "forward"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2014  Tobias Neumann
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

#ifndef __PLUGINS_COLLI_FORWARD_OMNI_DRIVE_MODE_H_
#define __PLUGINS_COLLI_FORWARD_OMNI_DRIVE_MODE_H_

#include "abstract_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ForwardOmniDriveModule : public AbstractDriveMode
{
 public:
  ForwardOmniDriveModule(Logger* logger, Configuration* config);
  ~ForwardOmniDriveModule();

  virtual void update();

 private:
  void calculate_rotation(float ori_alpha_target, float ori_alpha_next_target,
                          float dist_to_target, float angle_allowed_to_next_target);

  void calculate_translation(float dist_to_target, float ori_alpha_target, float dec_factor);
};

} // namespace fawkes

#endif
