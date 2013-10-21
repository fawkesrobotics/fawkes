
/***************************************************************************
 *  slow_forward_drive_mode.h - Implementation of drive-mode "slow forward"
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

#ifndef __PLUGINS_COLLI_SLOW_FORWARD_DRIVE_MODE_H_
#define __PLUGINS_COLLI_SLOW_FORWARD_DRIVE_MODE_H_

#include "abstract_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** This is the slow-forward drive-module. It is inherited from
 *    the abstract drive mode.
 */
class CSlowForwardDriveModule : public CAbstractDriveMode
{
 public:

  /** Constructor does only set the name of the slow forward drive mode.
   */
  CSlowForwardDriveModule(Logger* logger, Configuration* config);


  /** Destructor does nothing, because nothing was created in here.
   */
  ~CSlowForwardDriveModule();


  /** This Routine is called. Afterwards the m_proposedTranslation and
   *    m_proposedRotation have to be filled. Here they are
   *    filled up to 1 m/s and M_PI rad/s.
   */
  virtual void Update();


 private:


  float SlowForward_Curvature( float dist_to_target, float dist_to_trajec, float alpha,
             float trans_0, float rot_0 );
  float SlowForward_Translation( float dist_to_target, float dist_to_front, float alpha,
         float trans_0, float rot_0, float rot_1 );

  float m_MaxTranslation, m_MaxRotation;

};

} // namespace fawkes

#endif
