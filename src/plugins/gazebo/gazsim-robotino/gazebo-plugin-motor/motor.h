/***************************************************************************
 *  motor.h - Plugin for controling a model through a simulated motor
 *
 *  Created: Wed Jan 29 16:08:32 2014
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
  class Motor : public ModelPlugin
  {
  public:
    //Constructor
    Motor();

    //Destructor
    ~Motor();

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
    //name of the motor and the communication channel
    std::string name_;


    //Motor Stuff:
    //Functions for recieving Messages (registerd via suscribers)
    void on_motor_move_msg(ConstVector3dPtr &msg);

    //Suscriber for MotorMove Interfaces from Fawkes
    transport::SubscriberPtr motor_move_sub_;


    //current movement commands:
    float vx_, vy_, vomega_;  
  };
}
