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
/* Description: This is the quadratic motor instructor for Colli-A*     */
/*                                                                      */
/* Author:   Stefan Jacobs                                              */
/* Contact:  <Stefan_J@gmx.de>                                          */
/*                                                                      */
/* DOC.: This module is a class for validity checks of drive            */
/*         commands and sets those things with respect to the physical  */
/*         borders of the robot.                                        */
/*       For this purpose the two functions CalculateRotation and       */
/*         CalculateTranslation are implemented linearily ;-)           */
/*                                                                      */
/*                                                                      */
/* last modified: $Date$                          */
/*            by: $Author$                                    */
/*                                                                      */
/* ******************************************************************** */


#include "quadratic_motor_instruct.h"

#include <interfaces/MotorInterface.h>
#include <logging/logger.h>
#include <config/config.h>
#include <utils/math/common.h>

#include <string>

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

using namespace std;


CQuadraticMotorInstruct::CQuadraticMotorInstruct( fawkes::MotorInterface* motor,
                                                  fawkes::MotorInterface* motor_des,
                                                  float frequency,
                                                  fawkes::Logger* logger,
                                                  fawkes::Configuration* config )
 : CBaseMotorInstruct( motor, motor_des, frequency, logger ),
   config_( config )
{
  logger_->log_info("CQuadraticMotorInstruct", "(Constructor): Entering");

  string cfg_prefix = "/plugins/colli/";

  basic_trans_acc = config_->get_float((cfg_prefix + "CQuadraticMotorInstruct_BASIC_TRANS_ACC").c_str());
  basic_trans_dec = config_->get_float((cfg_prefix +"CQuadraticMotorInstruct_BASIC_TRANS_DEC").c_str());
  basic_rot_acc   = config_->get_float((cfg_prefix +"CQuadraticMotorInstruct_BASIC_ROT_ACC").c_str());
  basic_rot_dec   = config_->get_float((cfg_prefix +"CQuadraticMotorInstruct_BASIC_ROT_DEC").c_str());

  logger_->log_info("CQuadraticMotorInstruct", "(Constructor): Exiting");
}


CQuadraticMotorInstruct::~CQuadraticMotorInstruct()
{
  logger_->log_info("CQuadraticMotorInstruct", "(Destructor): Entering");
  logger_->log_info("CQuadraticMotorInstruct", "(Destructor): Exiting");
}




/* Implementation of Calculate Translation Function.
 * These are dangerous! Take care while modifying. Only a minus sign too few
 *   or too much may result in non predictable motor behaviour!!!!
 * THIS FUNCTION IS THE LAST BORDER TO THE MOTOR, TAKE CARE AND PAY ATTENTION!!!
 */
float CQuadraticMotorInstruct::CalculateTranslation( float currentTranslation,
                 float desiredTranslation,
                 float time_factor )
{
  float execTranslation = 0.0;

  if (desiredTranslation < currentTranslation) {

    if (currentTranslation > 0.0) {
      // decrease forward speed
      execTranslation = currentTranslation - basic_trans_dec - ((sqr( fabs(currentTranslation) + 1.0 ) * basic_trans_dec) / 8.0);
      execTranslation = max( execTranslation, desiredTranslation );

    } else if (currentTranslation < 0.0) {
      // increase backward speed
      execTranslation = currentTranslation - basic_trans_acc - ((sqr( fabs(currentTranslation) + 1.0 ) * basic_trans_acc) / 8.0);
      execTranslation = max( execTranslation, desiredTranslation );

    }  else {
      // currentTranslation == 0;
      execTranslation = max( -basic_trans_acc, desiredTranslation );
    }

  } else if (desiredTranslation > currentTranslation) {

    if (currentTranslation > 0.0) {
      // increase forward speed
      execTranslation = currentTranslation + basic_trans_acc + ((sqr( fabs(currentTranslation) + 1.0 ) * basic_trans_acc) / 8.0);
      execTranslation = min( execTranslation, desiredTranslation );

    } else if (currentTranslation < 0.0) {
      // decrease backward speed
      execTranslation = currentTranslation + basic_trans_dec + ((sqr( fabs(currentTranslation) + 1.0 ) * basic_trans_dec) / 8.0);
      execTranslation = min( execTranslation, desiredTranslation );

    } else {
      // currentTranslation == 0
      execTranslation = min( basic_trans_acc, desiredTranslation );
    }

  } else {
    // nothing to change!!!
    execTranslation = desiredTranslation;
  }

  return execTranslation*time_factor;
}




/* Implementation of Calculate Rotation Function.
 * These are dangerous! Take care while modifying. Only a minus sign too few
 *   or too much may result in non predictable motor behaviour!!!!
 * THIS FUNCTION IS THE LAST BORDER TO THE MOTOR, TAKE CARE AND PAY ATTENTION!!!
 */
float CQuadraticMotorInstruct::CalculateRotation( float currentRotation,
              float desiredRotation,
              float time_factor  )
{
  float execRotation = 0.0;

  if (desiredRotation < currentRotation) {

    if (currentRotation > 0.0) {
      // decrease right rot
      execRotation = currentRotation - basic_rot_dec - ((sqr( fabs(currentRotation) + 1.0 ) * basic_rot_dec) / 8.0);
      execRotation = max( execRotation, desiredRotation );

    } else if (currentRotation < 0.0) {
      // increase left rot
      execRotation = currentRotation - basic_rot_acc - ((sqr( fabs(currentRotation) + 1.0 ) * basic_rot_acc) / 8.0);
      execRotation = max( execRotation, desiredRotation );

    } else {
      // currentRotation == 0;
      execRotation = max( -basic_rot_acc, desiredRotation );
    }

  } else if (desiredRotation > currentRotation) {
    if (currentRotation > 0.0) {
      // increase right rot
      execRotation = currentRotation + basic_rot_acc + ((sqr( fabs(currentRotation) + 1.0 ) * basic_rot_acc) / 8.0);
      execRotation = min( execRotation, desiredRotation );

    } else if (currentRotation < 0.0) {
      // decrease left rot
      execRotation = currentRotation + basic_rot_dec + ((sqr( fabs(currentRotation) + 1.0 ) * basic_rot_dec) / 8.0);
      execRotation = min( execRotation, desiredRotation );

    } else {
      // currentRotation == 0
      execRotation = min( basic_rot_acc, desiredRotation );
    }

  } else {
    // nothing to change!!!
    execRotation = desiredRotation;
  }

  return execRotation*time_factor;
}

} // namespace fawkes
