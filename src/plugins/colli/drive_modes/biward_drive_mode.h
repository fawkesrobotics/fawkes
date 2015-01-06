
/***************************************************************************
 *  slow_biward_drive_mode.h - Implementation of drive-mode "slow forward + backward"
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

#ifndef __PLUGINS_COLLI_BIWARD_DRIVE_MODE_H_
#define __PLUGINS_COLLI_BIWARD_DRIVE_MODE_H_

#include "abstract_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class ForwardDriveModule;
class BackwardDriveModule;

class BiwardDriveModule : public AbstractDriveMode
{
 public:
  BiwardDriveModule( ForwardDriveModule*  forward,
                     BackwardDriveModule* backward,
                     Logger* logger,
                     Configuration* config);
  ~BiwardDriveModule();

  void update();

 private:
  ForwardDriveModule*  mod_forward_;
  BackwardDriveModule* mod_backward_;

  int   count_forward_;
};

} // namespace fawkes

#endif
