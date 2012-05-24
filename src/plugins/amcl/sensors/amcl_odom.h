/***************************************************************************
 *  amcl_odom.h: Odometry sensor model for AMCL
 *
 *  Created: Thu May 24 18:51:17 2012
 *  Copyright  2000  Brian Gerkey
 *             2000  Kasper Stoy
 *             2012  Tim Niemueller [www.niemueller.de]
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

/*  From:
 *  Player - One Hell of a Robot Server (LGPL)
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 */
///////////////////////////////////////////////////////////////////////////
// Desc: Odometry sensor model for AMCL
// Author: Andrew Howard
// Date: 17 Aug 2003
///////////////////////////////////////////////////////////////////////////

#ifndef AMCL_ODOM_H
#define AMCL_ODOM_H

#include "amcl_sensor.h"
#include "../pf/pf_pdf.h"

/// @cond EXTERNAL

namespace amcl
{

typedef enum
{
  ODOM_MODEL_DIFF,
  ODOM_MODEL_OMNI
} odom_model_t;

// Odometric sensor data
class AMCLOdomData : public AMCLSensorData
{
  // Odometric pose
  public: pf_vector_t pose;

  // Change in odometric pose
  public: pf_vector_t delta;
};


// Odometric sensor model
class AMCLOdom : public AMCLSensor
{
  // Default constructor
  public: AMCLOdom();

  public: void SetModelDiff(double alpha1, 
                            double alpha2, 
                            double alpha3, 
                            double alpha4);

  public: void SetModelOmni(double alpha1, 
                            double alpha2, 
                            double alpha3, 
                            double alpha4,
                            double alpha5);

  // Update the filter based on the action model.  Returns true if the filter
  // has been updated.
  public: virtual bool UpdateAction(pf_t *pf, AMCLSensorData *data);

  // Current data timestamp
  private: double time;
  
  // Model type
  private: odom_model_t model_type;

  // Drift parameters
  private: double alpha1, alpha2, alpha3, alpha4, alpha5;
};


}

/// @endcond

#endif
