/***************************************************************************
 *  amcl_sensor.cpp: AMCL sensor
 *
 *  Created: Thu May 24 18:52:54 2012
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
// Desc: AMCL sensor
// Author: Andrew Howard
// Date: 6 Feb 2003
///////////////////////////////////////////////////////////////////////////


#include "amcl_sensor.h"

using namespace amcl;

/// @cond EXTERNAL

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLSensor::AMCLSensor()
{
  return;
}

AMCLSensor::~AMCLSensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool AMCLSensor::UpdateAction(pf_t *pf, AMCLSensorData *data)
{
  return false;
}


////////////////////////////////////////////////////////////////////////////////
// Initialize the filter
bool AMCLSensor::InitSensor(pf_t *pf, AMCLSensorData *data)
{
  return false;
}


////////////////////////////////////////////////////////////////////////////////
// Apply the sensor model
bool AMCLSensor::UpdateSensor(pf_t *pf, AMCLSensorData *data)
{
  return false;
}


#ifdef INCLUDE_RTKGUI

////////////////////////////////////////////////////////////////////////////////
// Setup the GUI
void AMCLSensor::SetupGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig)
{
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the GUI
void AMCLSensor::ShutdownGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig)
{
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Draw sensor data
void AMCLSensor::UpdateGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig, AMCLSensorData *data)
{
  return;
}

#endif

/// @endcond

