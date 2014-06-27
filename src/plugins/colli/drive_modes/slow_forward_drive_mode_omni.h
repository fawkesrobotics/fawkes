
/***************************************************************************
 *  slow_forward_drive_mode.h - Implementation of drive-mode "slow forward"
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

#ifndef __PLUGINS_COLLI_SLOW_FORWARD_DRIVE_MODE_OMNI_H_
#define __PLUGINS_COLLI_SLOW_FORWARD_DRIVE_MODE_OMNI_H_

#include "abstract_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CSlowForwardOmniDriveModule : public CAbstractDriveMode
{
 public:

  CSlowForwardOmniDriveModule(Logger* logger, Configuration* config);
  ~CSlowForwardOmniDriveModule();

  virtual void Update();


 private:

  float m_MaxTranslation, m_MaxRotation;

  void calculateRotation(float ori_alpha_target, float ori_alpha_next_target, float dist_to_target);
};

} // namespace fawkes

#endif
