
/***************************************************************************
 *  backward_drive_mode.h - Implementation of drive-mode "backward"
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
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

#ifndef __PLUGINS_COLLI_BACKWARD_DRIVE_MODE_H_
#define __PLUGINS_COLLI_BACKWARD_DRIVE_MODE_H_

#include "abstract_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CBackwardDriveModule : public CAbstractDriveMode
{
 public:

  CBackwardDriveModule(Logger* logger, Configuration* config);
  ~CBackwardDriveModule();

  void Update();


 private:

  float Backward_Translation ( float dist_to_target, float dist_to_front, float alpha,
           float trans_0, float rot_0, float rot_1 );

  float Backward_Curvature( float dist_to_target, float dist_to_trajec, float alpha,
        float trans_0, float rot_0 );

  float m_MaxTranslation, m_MaxRotation;

};

} // nanespace fawkes

#endif
