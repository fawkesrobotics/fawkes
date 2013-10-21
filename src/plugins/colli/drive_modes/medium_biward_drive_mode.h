
/***************************************************************************
 *  medium_biward_drive_mode.h - Implementation of drive-mode "medium forward + backward"
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

#ifndef __PLUGINS_COLLI_MEDIUM_BIWARD_DRIVE_MODE_H_
#define __PLUGINS_COLLI_MEDIUM_BIWARD_DRIVE_MODE_H_

#include "abstract_drive_mode.h"
#include "medium_forward_drive_mode.h"
#include "medium_backward_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CMediumBiwardDriveModule : public CAbstractDriveMode
{
public:

  /** Constructor.
   */
  CMediumBiwardDriveModule( CMediumForwardDriveModule*  forward_module,
                            CMediumBackwardDriveModule* backward_module,
                            Logger* logger,
                            Configuration* config );


  /** Destructor. Does nothing, because nothing was created in this module.
   */
  ~CMediumBiwardDriveModule();


  /** This Routine is called. Afterwards the m_proposedTranslation and
   *    m_proposedRotation have to be filled. Here they are
   *    set to zero.
   */
  virtual void Update();


 private:

  float m_MaxTranslation, m_MaxRotation;

  CMediumForwardDriveModule*   m_pMediumForwardDriveModule;
  CMediumBackwardDriveModule*  m_pMediumBackwardDriveModule;

  int   m_CountForward;

};

} // namespace fawkes

#endif
