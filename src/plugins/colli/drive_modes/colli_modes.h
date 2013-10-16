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
/* $Id$  */
/*                                                                      */
/* Description: This is the enumeration defining the various colli      */
/*              modes which can be set via the colli_target_cleint      */
/*              interface                                               */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#ifndef _COLLI_MODES_H_
#define _COLLI_MODES_H_


/** Available Collimodes follow here!
 *
 *  Do use only the specific settings for your scenario.
 *
 *  Meaning: Slow modes for the _Floor_ and _Unknown Dynamics Surroundings_
 *           Moderate and fast only for robocup.
 *
 *    I have to test those first in dynamic office surroundings in reality!!
 *
 *
 *                                              Your dear colli author, Rudi
 *
 */
enum ColliModes
  {
    MovingNotAllowed = 0,                    // implemented

    CarefulForward          = 1050,
    SlowForward             = 1150,          // implemented
    ModerateForward         = 1250,          // implemented, robocup specific
    FastForward             = 1350,          // implemented, robocup specific

    CarefulAllowBackward    = 2050,
    SlowAllowBackward       = 2150,          // implemented
    ModerateAllowBackward   = 2250,          // implemented, robocup specific
    FastAllowBackward       = 2350,          // implemented, robocup specific

    CarefulBackward         = 3050,
    SlowBackward            = 3150,          // implemented
    ModerateBackward        = 3250,          // implemented, robocup specific
    FastBackward            = 3350,          // implemented, robocup specific

    ESCAPE                  = 9999,          // implemented

    SlowDribbleBall         = 4150,          // not implemented
    ModerateDribbleBall     = 4250,          // not implemented
    FastDribbleBall         = 4350,          // not implemented

    OVERRIDE                = 666,           // to use mopo-obj while colli is running

  };

#endif // _COLLI_MODES_H_
