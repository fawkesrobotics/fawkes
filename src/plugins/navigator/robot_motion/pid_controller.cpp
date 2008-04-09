 
/***************************************************************************
 *  pid_controller.cpp - PID controller
 *
 *  Generated: Wed April 03 00:31:38 20 
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

/** @class PidController plugins/navigator/robot_motion/pid_controller.h
 * A PID Controller.
 * To use, first set the P, I, and D coefficients with SetCoefficients(), 
 * then begin recording error values with Record(). Use GetOutput()
 * to calculate the current output of the controller.
 */

#include <plugins/navigator/robot_motion/pid_controller.h>
#include <iostream>

using namespace std;

/** Set the PID coefficients.
 * @param p_coefficient the P coefficient
 * @param i_coefficient the I coefficient
 * @param d_coefficient the D coefficient
 */
void
PidController::set_coefficients(float p_coefficient, 
			       float i_coefficient, 
			       float d_coefficient)
{ 
  m_pcoefficient = p_coefficient;
  m_icoefficient = i_coefficient;
  m_dcoefficient = d_coefficient;
}
  
/** Reset our controller to contain no error terms. */
void 
PidController::clear()
{
  m_current_index       = -1;
  m_previous_index      = -1;
  m_num_errors_recorded =  0;
  m_current_integral    = 0.0f;

  for (unsigned int i = 0; i < __PID_CONTROL_NUM_ERROR_SLOTS_; ++i)
    {
      m_error[i]	= 0.0f;
      m_timestep[i]	= 0.0f;
    }
}

/** Record an error term and its associated timestep.
 * Update our current integral value as well. The integral of a
 * function is the area under its curve. In this case, we have NUM_ERROR_SLOTS
 * samples of values from our error function, and wish to approximate its 
 * integral. We see that we have a series of samples and their associated 
 * timesteps. Each sample/timestep pair can represent a rectangle of area
 * on our graph. By summing together these areas, we can approximate the integral
 * of our error function.
 *
 * The limit as the size of the timesteps approach zero is the actual integral
 * of the error function.
 *
 * For more information about numerical integration, please see chapter 4 of
 * the Numerical Recipes books, available online at http://www.nr.com/
 *
 * @param error the current error (desired - actual)
 * @param timestep the time that passed since the last call in seconds
 */
void 
PidController::record(float error, float timestep)
{
  m_previous_index = m_current_index;
  m_current_index  = (m_current_index + 1) % __PID_CONTROL_NUM_ERROR_SLOTS_;

  if (m_num_errors_recorded == __PID_CONTROL_NUM_ERROR_SLOTS_)
    {
      m_current_integral -= m_error[m_current_index] * m_timestep[m_current_index];
      --m_num_errors_recorded;
    }

  ++m_num_errors_recorded;

  m_error[m_current_index]     = error;
  m_timestep[m_current_index]  = timestep;
  m_current_integral          += error * timestep;
}



/** Returns the last error term recorded. 
 * @return the last recorded error term
 */
float 
PidController::get_error()
{
  if (m_num_errors_recorded >= 1)
    {
      // ASSERT((m_current_index >= 0) && (m_current_index < __PID_CONTROL_NUM_ERROR_SLOTS_));
      
      return m_error[m_current_index];
    }
  else
    {
      return 0.0f;
    }
}

/** Returns the integral of the last NUM_ERROR_SLOTS error terms recorded.
 * See PidController::record() for how m_current_integral is calculated.
 * Alternately, see PidController::calculate_integral_directly() for
 * how to calculate the integral from scratch.
 *
 * @return the integral of the last NUM_ERROR_SLOTS error terms recorded
 */
float 
PidController::get_error_integral()
{
  return m_current_integral;
}

/** Returns the current instantaneous derivative the error.
 * The derivative of a variable is its rate of change. We can estimate its
 * current rate of change by dividing the difference between two successive 
 * samples of the variable by the time elapsed between them.
 *
 * The limit as the size of the timesteps approaches zero is the actual derivative
 * of the error function.
 *
 * @return the current instantaneous derivative the error
 */
float 
PidController::get_error_derivative()
{
  if (m_num_errors_recorded >= 2)
    {
      // ASSERT((m_current_index	>= 0) && (m_current_index < __PID_CONTROL_NUM_ERROR_SLOTS_));
      // ASSERT((m_previous_index >= 0) && (m_previous_index < __PID_CONTROL_NUM_ERROR_SLOTS_));
      
      float difference	  = m_error[m_current_index] - m_error[m_previous_index];
      float time_interval = m_timestep[m_current_index];

      if ( time_interval > 0.001f )
	{
	  return (difference / time_interval);
	}
      else
	{
	  return 999999.0f;
	}
    }
  else
    {
      return 0.0f;
    }
}

/** Calculates the current output of our controller.
 * @return the current output of our controller
 */
float 
PidController::get_output()
{
  return ( m_pcoefficient * get_error() + 
	   m_icoefficient * get_error_integral() + 
	   m_dcoefficient * get_error_derivative() );
}

/** Used for debugging to determine the amount of floating point
 * error that has accumulated in m_current_integral, by calculating
 * the current integral from scratch.
 *
 * See PidController::record() for an explanation of approximating
 * the integral of a variable.
 *
 * @return the value of the current integral
 */
float 
PidController::calculate_integral_directly()
{
  float integral = 0.0f;

  for (unsigned int i = 0; i < m_num_errors_recorded; ++i)
    {
      integral += (m_error[i] * m_timestep[i]);
    }

  return integral;
}



/** Debug printing of our current state. */
void 
PidController::dump_state()
{
  std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";
  std::cout << ">>> Num errors recorded: " << m_num_errors_recorded 
	    << ". Current index: " << m_current_index 
	    << ". Previous index: " <<  m_previous_index << std::endl;
  for (int i = 0; i < __PID_CONTROL_NUM_ERROR_SLOTS_; i++)
    {
      std::cout << ">>> Slot " 
		<< i << ": Error: " << m_error[i] 
		<< " Timestep: " << m_timestep[i] << "\n";
    }
  std::cout << ">>>\n";
  std::cout << ">>> Current P term: Coefficient " << m_pcoefficient 
	    << " * Error      " << get_error() << " = " 
	    << m_pcoefficient * get_error() << std::endl;
  
  std::cout << ">>> Current I term: Coefficient " << m_icoefficient 
	    << " * Integral   " << get_error_integral() 
	    << " = " << m_icoefficient * get_error_integral() << std::endl;
  
  std::cout << ">>> Current D term: Coefficient " << m_dcoefficient
	    << " * Derivative " << get_error_derivative() 
	    << " = " << m_dcoefficient * get_error_derivative() << std::endl;
  std::cout << ">>>\n";
  
  std::cout << ">>> Final output: " << get_output() << "\n";
  std::cout << ">>> Integral: " << get_error_integral() << ". Calculated directly: " << calculate_integral_directly() << "\n";
  std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";
}
