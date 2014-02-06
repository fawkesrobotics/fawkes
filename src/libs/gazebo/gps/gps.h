/***************************************************************************
 *  gps.h - provides ground truth position
 *
 *  Created: Tue Feb 04 15:05:07 2014
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

namespace gazebo
{   
  class Gps : public ModelPlugin
  {
  public:
    //Constructor
    Gps();

    //Destructor
    ~Gps();

    //Overridden ModelPlugin-Functions
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    virtual void OnUpdate(const common::UpdateInfo &);
    virtual void Reset();

  private:
    // Pointer to the model
    physics::ModelPtr model_;
    // Pointer to the update event connection
    event::ConnectionPtr update_connection_;
    //Node for communication to fawkes
    transport::NodePtr node_;
    //name of the gps and the communication channel
    std::string name_;

    //time variable to send in intervals
    double last_sent_time_;

    //Gps Stuff:
    //Functions for sending ionformation to fawkes:
    void send_position();

    //Publisher for GyroAngle
    transport::PublisherPtr gps_pub_;
  };
}
