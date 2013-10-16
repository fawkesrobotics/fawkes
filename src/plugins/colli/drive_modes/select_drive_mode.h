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
/* $Id$    */
/*                                                                      */
/* Description: This is an abstract drive module interface of Colli-A*  */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This is the select-drive-mode module.                          */
/*       Call class for all other drive modes.                          */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */

#ifndef _COLLI_SELECT_DRIVE_MODE_H_
#define _COLLI_SELECT_DRIVE_MODE_H_

//#include "abstract_drive_mode.h"

#include <vector>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class CAbstractDriveMode;
class NavigatorInterface;
class MotorControl;
class Laser;
class Logger;
class Configuration;


/// This class selects the correct drive mode and calls the appopriate drive component
class CSelectDriveMode
{
public:

  /// laser + target wird hier gebraucht
  CSelectDriveMode( MotorControl* motor,
                    Laser* laser,
                    NavigatorInterface* target,
                    Logger* logger,
                    Configuration* config );


  ///
  ~CSelectDriveMode( );


  /** Set local target point before update!
   */
  void SetLocalTarget( float localTargetX, float localTargetY );


  /** Set local trajectory point before update!
   */
  void SetLocalTrajec( float localTrajecX, float localTrajecY );


  /** Has to be called before the proposed values are called.
   */
  void Update( bool escape = false );


  /** Returns the proposed translation.
   *  After an update.
   */
  float GetProposedTranslation();


  /** Returns the proposed rotation.
   *  After an update.
   */
  float GetProposedRotation();

private:

  // local pointers to bb client objects
  fawkes::NavigatorInterface*      m_pColliTarget;
  MotorControl*                    m_pMotor;       // USE ONLY AS GETTER!!!
  Laser*                           m_pLaser;

  fawkes::Logger* logger_;
  fawkes::Configuration* config_;

  // Vector of drive modes
  std::vector< fawkes::CAbstractDriveMode * > m_vDriveModeList;


  // local copies of current local target values
  float m_LocalTargetX, m_LocalTargetY;
  float m_LocalTrajecX, m_LocalTrajecY;

  // local copies of the proposed values
  float m_ProposedTranslation;
  float m_ProposedRotation;

  // an escape flag
  int m_EscapeFlag;


  /* ************************************************************************ */
  /* PRIVATE METHODS                                                          */
  /* ************************************************************************ */

};

} // namespace fawkes

#endif
