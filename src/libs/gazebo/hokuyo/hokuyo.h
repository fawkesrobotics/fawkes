/***************************************************************************
 *  hokuyo.h - Plugin for getting laser sensor data from the simulation
 *
 *  Created: Wed Jan 29 16:56:04 2014
 *  Copyright  2014  Frederik Zwilling
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <gazebo/transport/transport.hh>
#include <list>
#include <string.h>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

namespace gazebo
{   
  class Hokuyo : public ModelPlugin
  {
  public:
    //Constructor
    Hokuyo();

    //Destructor
    ~Hokuyo();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual void Reset();


    //what happens if the sensor has new laser data
    void on_new_laser_scans();

  private:
    // Pointer to the model
    physics::ModelPtr model_;
    //Node for communication to fawkes
    transport::NodePtr node_;
    //name of the hokuyo and the communication channel
    std::string name_;

    //connection of the sensor
    event::ConnectionPtr new_laser_scans_connection_;
    //Pointer to the hokuyo sensor
    sensors::RaySensorPtr parent_sensor_;
    //Publisher for communication to fawkes
    transport::PublisherPtr laser_pub_;

  };
}
