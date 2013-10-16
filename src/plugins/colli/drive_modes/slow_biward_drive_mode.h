//     A* Collision Avoidance Algorithm by Stefan Jacobs
//     Copyright (C) 2002  Stefan Jacobs <Stefan_J@gmx.de>
//
//     This program is free software; you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation; either version 2 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program; if not, write to the Free Software
//     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//


/*
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
  ©                                                                            ©
  ©                                            ####   ####           .-""-.    ©
  ©       # #                             #   #    # #    #         /[] _ _\   ©
  ©       # #                                 #    # #             _|_o_LII|_  ©
  © ,###, # #  ### ## ## ##   ###  ## ##  #   #    # #       ###  / | ==== | \ ©
  © #   # # # #   # ## ## #  #   #  ## #  #   ###### #      #     |_| ==== |_| ©
  © #   # # # ####  #  #  #  #   #  #  #  #   #    # #      ####   ||" ||  ||  ©
  © #   # # # #     #  #  #  #   #  #  #  #   #    # #    #    #   ||LI  o ||  ©
  © '###'# # # #### #  #  ##  ### # #  ## ## #      # ####  ###    ||'----'||  ©
  ©                                                               /__|    |__\ ©
  ©                                                                            ©
  ©º°¨¨°º©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©©º°¨¨°º©º°¨¨°º©
*/


/* ******************************************************************** */
/*                                                                      */
/* $Id$ */
/*                                                                      */
/* Description: This is the slow forward and backward drive module      */
/*                interface of Colli-A*                                 */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the slow forward and backward drive module.            */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_SLOW_BIWARD_DRIVE_MODE_H_
#define _COLLI_SLOW_BIWARD_DRIVE_MODE_H_

#include "abstract_drive_mode.h"
#include "slow_forward_drive_mode.h"
#include "slow_backward_drive_mode.h"

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** This is the slow-biward drive-module. It is inherited from
 *    the abstract drive mode and uses the other both modes.
 *    If the target is in front, it drives forward to the target,
 *    else it drives backward to the target.
 */
class CSlowBiwardDriveModule : public CAbstractDriveMode
{
 public:

  /** Constructor does set name and gets the two other drive
   *    modes.
   */
  CSlowBiwardDriveModule( CSlowForwardDriveModule*  slow_forward,
                          CSlowBackwardDriveModule* slow_backward,
                          Logger* logger,
                          Configuration* config);


  /** Destructor does nothing, because nothing is created in here.
   */
  ~CSlowBiwardDriveModule();


  /** This Routine is called. Afterwards the m_proposedTranslation and
   *    m_proposedRotation have to be filled. Here they are
   *    filled up in between -1 m/s to 1 m/s and M_PI rad/s.
   */
  void Update();


 private:

  CSlowForwardDriveModule*  m_pSlowForwardDriveModule;
  CSlowBackwardDriveModule* m_pSlowBackwardDriveModule;

  float m_MaxTranslation, m_MaxRotation;


  int   m_CountForward;

};

} // namespace fawkes

#endif
