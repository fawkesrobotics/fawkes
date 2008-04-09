 
/***************************************************************************
 *  pid_controller.h - PID controller
 *
 *  Generated: Wed April 09 15:03:47 2008 
 *  Copyright  2002-2003 Stefan Jacobs
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_NAVIGATOR_ROBOT_MOTION_CPIDCONTOLLER_H_
#define __PLUGINS_NAVIGATOR_ROBOT_MOTION_CPIDCONTOLLER_H_

#define __PID_CONTROL_NUM_ERROR_SLOTS_ 10

class PidController
{
 public:
  PidController()  { set_coefficients(0.0f, 0.0f, 0.0f); clear(); }
  ~PidController() {}
  
  /** Check whether all slots are occupied.
   * @return true if all slots are occupied
   */
  inline bool is_full()
  { return (m_num_errors_recorded == __PID_CONTROL_NUM_ERROR_SLOTS_); }

  void set_coefficients(float p_coefficient, 
			float i_coefficient, 
			float d_coefficient);

  void	clear();
  void	record(float error, float timestep);

  float	get_error();
  float	get_error_integral();
  float	get_error_derivative();
  
  float	get_output();
  
  void	dump_state();
  
 private:
  float	calculate_integral_directly();
  
  float	m_pcoefficient;	// Our current P coefficient
  float	m_icoefficient;	// Our current I coefficient
  float	m_dcoefficient;	// Our current D coefficient
  
  // The last NUM_ERROR_SLOTS error terms that have been recorded
  float	m_error[__PID_CONTROL_NUM_ERROR_SLOTS_];
  
  // The last NUM_ERROR_SLOTS timesteps that have been recorded
  float	m_timestep[__PID_CONTROL_NUM_ERROR_SLOTS_];  
  

  // Index into m_Error[] anmd m_Timestep[] of the last recorded error
  int m_current_index;	   

  // Index into m_Error[] anmd m_Timestep[] of the next-to-last recorded error
  int m_previous_index;	   

  // Number of error values that have been recorded so far. Between 0 and NUM_ERROR_SLOTS.
  unsigned int m_num_errors_recorded; 

  // The current value of our integral term
  float	m_current_integral;	
};

#endif /* __PLUGINS_NAVIGATOR_ROBOT_MOTION_CPIDCONTOLLER_H_ */
