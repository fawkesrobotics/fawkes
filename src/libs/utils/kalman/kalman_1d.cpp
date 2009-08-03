/***************************************************************************
 *  kalman_1d.cpp - Kalman filter (one dimensional)
 *
 *  Created: Mon Nov 10 2008
 *  Copyright  2008  Bahram Maleki-Fard
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <utils/kalman/kalman_1d.h>
#include <math.h>

namespace fawkes {

/** @class KalmanFilter1D <utils/kalman/kalman_1d.h>
 * One-dimensional Kalman filter implementation for single-precision floats.
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param noise_x Transition noise, by default 1.0.
 * @param noise_z Sensor noise, by default 1.0.
 * @param mu Initial mu, by default 0.0.
 * @param sig Standard deviation, by default 1.0.
 */
KalmanFilter1D::KalmanFilter1D(float noise_x, float noise_z, float mu, float sig)
{
  __noise_x = noise_x;
  __noise_z = noise_z;
  __mu      = mu;
  __sig     = sig;
}

/** Destructor. */
KalmanFilter1D::~KalmanFilter1D()
{
}

/** Filters an observation. The internal mean and deviation are updated.
 * @param observe The observation.
 */
void
KalmanFilter1D::filter(float observe)
{
  float help = __sig*__sig + __noise_x*__noise_x + __noise_z*__noise_z;
  __mu  = ((__sig*__sig + __noise_x*__noise_x) * observe + __noise_z*__noise_z*__mu) / help;
  __sig = sqrt( (__sig*__sig + __noise_x*__noise_x)*__noise_z*__noise_z / help );
}


/** Filters an observation. The resulting mu and sig are not only stored
 * internally, but also in the given parameters mean and deviation.
 * @param observe The observation.
 * @param mu The mean (out parameter).
 * @param sig The deviation (out parameter)
 */
void
KalmanFilter1D::filter(float observe, float& mu, float& sig)
{
  mu  = __mu;
  sig = __sig;
}

/** Predicts the next position based on the past observations. Equivalent
 * to predict(0.0), i.e. velocity 0.0.
 * @return predicted value
 */
float
KalmanFilter1D::predict() const
{
  return predict(0.0);
}


/** Predicts the next position based on the past observations. Equivalent
 * to predict(vel, 1, 0.0). 
 * @param vel The velocity of the object, 0.0 by default.
 * @return predicted value
 */
float
KalmanFilter1D::predict(float vel) const
{
  return predict(vel, 1, 0.0);
}


/** Predicts the next position based on the past observations.
 * @param vel The velocity of the object.
 * @param steps The steps to look into the future.
 * @param noise_z Sensor noise.
 * @return predicted value
 */
float
KalmanFilter1D::predict(float vel, int steps, float noise_z) const
{
  return predict(__mu, vel, steps, noise_z);
}

/** Predicts the next position based on the past observations.
 * @param mu Explicitely 
 * @param vel The velocity of the object, 0.0 by default.
 * @param steps The steps to look into the future, 1 by default.
 * @param noise_z Sensor noise.
 * @return predicted value
 */
float
KalmanFilter1D::predict(float mu, float vel, int steps, float noise_z) const
{
  return mu + steps * (vel + noise_z);
}

}

