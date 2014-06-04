
/***************************************************************************
 *  robo_motorcontrol.h - Motor-control wrapper
 *
 *  Created: Fri Oct 18 15:16:23 2013
 *  Copyright  2002  Stefan Jacobs
 *             2013  Bahram Maleki-Fard
 *             2014  Tobias Neumann
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

#ifndef __PLUGINS_COLLI_UTILS_ROB_ROBO_MOTORCONTROL_H_
#define __PLUGINS_COLLI_UTILS_ROB_ROBO_MOTORCONTROL_H_

namespace fawkes
{
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

class MotorInterface;

class MotorControl
{

 public:

  /** Constructor.
   *  @param motor The MotorInterface.
   */
  MotorControl( fawkes::MotorInterface* motor );

  /** Desctructor. */
  ~MotorControl();


  /** GetCurrentX.
   *  This method returns current Odometry x-value.
   *  @return float is the current x coordinate.
   */
  float GetCurrentX();

  /** GetCurrentY.
   *  This method returns current Odometry y-value.
   *  @return float is the current y coordinate.
   */
  float GetCurrentY();

  /** GetCurrentOri.
   *  This method returns current Odometry ori-value.
   *  @return float is the current ori value.
   */
  float GetCurrentOri();

  /** GetMotorDesiredTranslationX.
   *  This method returns current desired motor x translation speed.
   *  @return float is the current desired motor translation speed.
   */
  float GetMotorDesiredTranslationX();

  /** GetMotorDesiredTranslationY.
   *  This method returns current desired motor y translation speed.
   *  @return float is the current desired motor translation speed.
   */
  float GetMotorDesiredTranslationY();

  /** GetMotorDesiredRotation.
   *  This method returns current desired motor rotation speed.
   *  @return float is the current desired motor rotation speed.
   */
  float GetMotorDesiredRotation();

  /** GetMotorCurrentTranslation.
   *  This method returns current (noisy) motor translation speed.
   *  @return float is the current (noisy) motor translation speed.
   */
  float GetMotorCurrentTranslation();

  /** GetMotorCurrentTranslationX.
   *  This method returns current (noisy) motor x translation speed.
   *  @return float is the current (noisy) motor translation speed.
   */
  float GetMotorCurrentTranslationX();

  /** GetMotorCurrentTranslationY.
   *  This method returns current (noisy) motor y translation speed.
   *  @return float is the current (noisy) motor translation speed.
   */
  float GetMotorCurrentTranslationY();

  /** GetMotorCurrentRotation.
   *  This method returns current (noisy) motor rotation speed.
   *  @return float is the current (noisy) motor rotation speed.
   */
  float GetMotorCurrentRotation();

  /** GetUserDesiredTranslationX.
   *  This method returns current desired user translation speed.
   *    The difference to the functions on top is that the user
   *    desired settings are not yet sended to the motor.
   *  @return float is the current desired user translation speed.
   */
  float GetUserDesiredTranslationX();

  /** GetUserDesiredTranslation.
   *  This method returns current desired user translation speed.
   *    The difference to the functions on top is that the user
   *    desired settings are not yet sended to the motor.
   *  @return float is the current desired user translation speed.
   */
  float GetUserDesiredTranslationY();

  /** GetUserDesiredRotation.
   *  This method returns current desired user rotation speed.
   *    The difference to the functions on top is that the user
   *    desired settings are not yet sended to the motor.
   *  @return float is the current desired user rotation speed.
   */
  float GetUserDesiredRotation();

  /** GetMovingAllowed.
   *  This method returns, if moving is currently allowed.
   *  @return bool if moving is currently allowed.
   */
  bool GetMovingAllowed();


  /** SetDesiredTranslationX.
   *  This method sets the current user desired translation speed.
   *  @param speed is a float containing x translation speed in m/s.
   */
  void SetDesiredTranslationX( float speed );

  /** SetDesiredTranslation.
   *  This method sets the current user desired translation speed.
   *  @param speed is a float containing y translation speed in m/s.
   */
  void SetDesiredTranslationY( float speed );

  /** SetDesiredRotation.
   *  This method sets the current user desired rotation speed.
   *  @param ori is a float containing rotaion speed in rad/s.
   */
  void SetDesiredRotation( float ori );

  /** SendCommand.
   *  This method is the final sending command. Here the blackboard gets
   *    informed about the users desired intentions.
   * @return true if successful, false otherwise
   */
  bool SendCommand( );




 private:

  float m_MotorControlDesiredTranslationX;  // what translation the next set command realizes
  float m_MotorControlDesiredTranslationY;  // what translation the next set command realizes
  float m_MotorControlDesiredRotation;      // what rotationtion the next set command realizes

  fawkes::MotorInterface* m_pMopo;      // Interface with motor-values
};

} // namespace fawkes

#endif
