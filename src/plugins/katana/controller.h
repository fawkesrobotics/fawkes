
/***************************************************************************
 *  controller.h - Controller class for katana arm
 *
 *  Created: Tue Jan 03 11:40:31 2012
 *  Copyright  2012  Bahram Maleki-Fard, AllemaniACs RoboCup Team
 *
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

#ifndef __PLUGINS_KATANA_CONTROLLER_H_
#define __PLUGINS_KATANA_CONTROLLER_H_

#include <vector>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class KatanaController <plugins/katana/controller.h>
 * Abstract class for a Neuronics Katana controller.
 * @author Bahram Maleki-Fard
 */
class KatanaController
{
 public:
  /** Virtual empty destructor. */
  virtual ~KatanaController() {}

  // setup
  /** Initialize controller */
  virtual void init() = 0;

  /** Set maximum velocity.
  * @param vel velocity
  */
  virtual void set_max_velocity(unsigned int vel) = 0;



  // status checking
  /** Check if movement is final.
  * @return movement is final
  */
  virtual bool final() = 0;

  /** Check if controller provides joint angle values.
  * @return can get angle values
  */
  virtual bool joint_angles() = 0;

  /** Check if controller provides joint encoder values.
  * @return can get encoder values
  */
  virtual bool joint_encoders() = 0;



  // commands
  /** Calibrate the arm. */
  virtual void calibrate() = 0;

  /** Turn on arm/motors. */
  virtual void turn_on() = 0;

  /** Turn off arm/motors. */
  virtual void turn_off() = 0;

  /** Stop movement immediately. */
  virtual void stop() = 0;

  /** Store current coordinates of endeeffctor.
  * @param refresh fetch new joint data from device (update data in controller library).
  * No need to set to 'true' if 'read_motor_data()' is being called regularly.
  */
  virtual void read_coordinates(bool refresh = false) = 0;

  /** Read motor data of currently active joints from device into controller libray. */
  virtual void read_motor_data() = 0;

  /** Read all sensor data from device into controller libray. */
  virtual void read_sensor_data() = 0;

  /** Open Gripper.
  * @param blocking Is this a blocking call?
  */
  virtual void gripper_open(bool blocking = false) = 0;

  /** Close Gripper.
  * @param blocking Is this a blocking call?
  */
  virtual void gripper_close(bool blocking = false) = 0;

  /** Move endeffctor to given coordinates.
  * @param x translation on x-axis.
  * @param y translation on y-axis.
  * @param z translation on z-axis.
  * @param phi 1st rotation of euler-ZXZ-rotation
  * @param theta 2nd rotation of euler-ZXZ-rotation
  * @param psi 3rd rotation of euler-ZXZ-rotation
  * @param blocking Is this a blocking call?
  */
  virtual void move_to(float x, float y, float z, float phi, float theta, float psi, bool blocking = false) = 0;

  /** Move joints to encoder values.
  * @param encoders vector containing encoder values for all joints.
  * @param blocking Is this a blocking call?
  */
  virtual void move_to(std::vector<int> encoders, bool blocking = false) = 0;

  /** Move joints to angle values.
  * @param angles vector containing angle values for all joints.
  * @param blocking Is this a blocking call?
  */
  virtual void move_to(std::vector<float> angles, bool blocking = false) = 0;

  /** Move single joint/motor to encoder value.
  * @param id id of the joint/motor.
  * @param enc target encoder value.
  * @param blocking Is this a blocking call?
  */
  virtual void move_motor_to(unsigned short id, int enc, bool blocking = false) = 0;

  /** Move single joint/motor to angle value.
  * @param id id of the joint/motor.
  * @param angle target angle value.
  * @param blocking Is this a blocking call?
  */
  virtual void move_motor_to(unsigned short id, float angle, bool blocking = false) = 0;

  /** Move single joint/motor by encoder value (i.e. increase/decrease).
  * @param id id of the joint/motor.
  * @param enc increase/decrease by encoder value.
  * @param blocking Is this a blocking call?
  */
  virtual void move_motor_by(unsigned short id, int enc, bool blocking = false) = 0;

  /** Move single joint/motor by angle value (i.e. increase/decrease).
  * @param id id of the joint/motor.
  * @param angle increase/decrease by angle value.
  * @param blocking Is this a blocking call?
  */
  virtual void move_motor_by(unsigned short id, float angle, bool blocking = false) = 0;




  // getters
  /** Get x-coordinate of latest endeffector position.
   * Call 'read_coordinates()' to read latest position.
   * @return x-coordinate
   */
  virtual double x() = 0;

  /** Get y-coordinate of latest endeffector position.
   * Call 'read_coordinates()' to read latest position.
   * @return y-coordinate
   */
  virtual double y() = 0;

  /** Get z-coordinate of latest endeffector position.
   * Call 'read_coordinates()' to read latest position.
   * @return z-coordinate
   */
  virtual double z() = 0;

  /** Get x-coordinate of latest endeffector position.
   * Call 'read_coordinates()' to read latest position.
   * @return x-coordinate
   */

  /** Get phi-rotation of latest endeffector orientation.
   * Call 'read_coordinates()' to read latest orientation.
   * @return phi-rotation (1st rotation of euler-ZXZ-rotation)
   */
  virtual double phi() = 0;

  /** Get theta-rotation of latest endeffector orientation.
   * Call 'read_coordinates()' to read latest orientation.
   * @return theta-rotation (2nd rotation of euler-ZXZ-rotation)
   */
  virtual double theta() = 0;

  /** Get psi-rotation of latest endeffector orientation.
   * Call 'read_coordinates()' to read latest orientation.
   * @return psi-rotation (3rd rotation of euler-ZXZ-rotation)
   */
  virtual double psi() = 0;

  /** Get sensor values.
   * @param to vector to be filled with all available sensor values.
   * @param refresh refresh sensor data (call 'read_sensor_data')?
   */
  virtual void get_sensors(std::vector<int>& to, bool refresh = false) = 0;

  /** Get encoder values of joints/motors.
   * @param to vector to be filled with encoder values for active joints.
   * @param refresh refresh joints/motors data (call 'read_motor_data')?
   */
  virtual void get_encoders(std::vector<int>& to, bool refresh = false) = 0;

  /** Get angle values of joints/motors.
   * @param to vector to be filled with angle values for active joints.
   * @param refresh refresh joints/motors data (call 'read_motor_data')?
   */
  virtual void get_angles(std::vector<float>& to, bool refresh = false) = 0;


};


} // end of namespace fawkes

#endif