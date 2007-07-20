
/***************************************************************************
 *  motor_thread.cpp - Motor Thread
 *
 *  Generated: Son Jun 03 00:07:33 2007
 *  Copyright  2007  Martin Liebenberg
 *
 *  $Id$
 *
 ****************************************************************************/

/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */


#include <plugins/navigator/motor_thread.h>
#include <plugins/navigator/navigator_thread.h>
#include <interfaces/motor_interface.h>
#include <utils/math/angle.h>

#include <vmc/LayerClasses/VMC_API.h>
#include <vmc/SupportClasses/Enums.h>

#include <cmath>
#include <unistd.h>

#define NO_VMC

/* @class MotorThread plugins/navigator/motor_thread.h
 * The thread controlling the motors.
 * It gets some driving commands and calculates the rpms for
 * the wheels. It gets the real rpms from the motor controller as well.
 */
/* @var MotorThread::forward 
 * The forward command.
 */
/* @var MotorThread::sideward 
 * The sideward command.
 */
/* @var MotorThread::rotation
 * The rotation command.
 */
/* @var MotorThread::speed 
 * The speed command.
 */
 
/** Constructor. 
 * @param navigator_thread the navigator thread to add this class as notification listener
 */
MotorThread::MotorThread(NavigatorThread *navigator_thread)
  : Thread("MotorThread", Thread::OPMODE_CONTINUOUS)
{
  motor_interface = NULL;
  forward = 0;
  sideward = 0;
  rotation = 0;
  speed = 0;
  logger_modulo_counter = 0;
  navigator_thread->add_notification_listener(this);
  timeout_counter = 0;
}

/** Destructor. */
MotorThread::~MotorThread()
{
}

/** Initialize thread.
 * Here, the motor interface is opened.
 */
void
MotorThread::init()
{
  try 
    {
      motor_interface = interface_manager->open_for_writing<MotorInterface>("Motor");
    }
  catch (Exception& e)
    {
      logger->log_error("MotorThread", "Opening interface failed!");
      logger->log_error("MotorThread", e);
      throw;
    }
#ifndef NO_VMC
  apiObject = new VMC::VMC_API();
  
  apiObject->selectHardwareAdapter(VMC::RS232);

  if ( ! apiObject->selectDevice("/dev/ttyS0") ) {
    throw Exception("MotorThread failed to open serial device VMC");
  }
#endif  
}

void
MotorThread::finalize()
{
  logger->log_info("MotorThread", "Finalizing thread %s", name());
  
  try 
    {
      interface_manager->close(motor_interface);
    }
  catch (Exception& e)
    {
      logger->log_error("MotorThread", "Closing interface failed!");
      logger->log_error("MotorThread", e);
    }
    
#ifndef NO_VMC
  apiObject->CloseDevice();
  
  delete apiObject;
#endif
}

/** Is called if the navigator thread is started.
 * It memorizes the id of the navigator thread
 * and sets the navigator as the current controller in the interface.
 * @param thread the navigator thread
 */
void 
MotorThread::thread_started(Thread *thread)
{
  std::cout << "navigator thread id " << thread->current_thread_id() << std::endl;
  motor_interface->setControllerID(thread->current_thread_id());
  motor_interface->write();
}

/** Just implemented for properness. It's empty.
 * @param thread the navigator thread
 */
void 
MotorThread::thread_init_failed(Thread *thread)
{
}

/** Sets the driving command.
 * @param forward the forward command
 * @param sideward the sideward command
 * @param rotation the rotation command
 * @param speed the speed command
 */
void 
MotorThread::setCommand(double forward, double sideward, double rotation, double speed)
{
  this->forward = forward;
  this->sideward = sideward;
  this->rotation = rotation;
  this->speed = speed;
}

/** Here the driving commands are transformed to the RPMs
 *   for the three motors.
 */
void
MotorThread::loop()
{
  /*
    if ( motor_interface->msgq_size() > 0 ) {
    logger->log_error(name(), "We have %u messages", motor_interface->msgq_size());
    }*/
  if ( motor_interface->msgq_first_is<MotorInterface::JoystickMessage>() )
    {
      MotorInterface::JoystickMessage* msg = motor_interface->msgq_first<MotorInterface::JoystickMessage>();

      forward = msg->getCmdForward();
      
      sideward = msg->getCmdSideward();
      
      try 
        {
          rotation = msg->getCmdRotation() / (3 * config->get_float("navigator", "/motor_thread/joystick_rotation"));
        } 
      catch (Exception &e) 
        {
          logger->log_error("MotorThread", "loop()");
          logger->log_error("MotorThread", e);
          throw;
        }
      
      speed = msg->getCmdSpeed();
      
  
      /*
        if((++logger_modulo_counter %= 30) == 0)
        {
        logger->log_info("MotorThread", "if1 forward command: %f  sideward command: %f  rotation command: %f speed commdan: %f", 
        forward,
        sideward,
        rotation,
        speed);
        }
      */
      motor_interface->msgq_pop();
    }
  else if (/*!extern_control && */motor_interface->msgq_first_is<MotorInterface::NavigatorMessage>() )
    {
      MotorInterface::NavigatorMessage* msg = motor_interface->msgq_first<MotorInterface::NavigatorMessage>();

      forward = msg->getCmdForward();
      
      sideward = msg->getCmdSideward();
      
      rotation = msg->getCmdRotation() / 3;
      
      speed = msg->getCmdVelocity();
      
  
      /*  
      // if((++logger_modulo_counter % 30) == 0)
      {
      logger->log_info("MotorThread", "2 NavigatorMessage forward command: %f  sideward command: %f  roation command: %f speed commdan: %f", 
      forward,
      sideward,
      rotation,
      speed);
      }
      */
      motor_interface->msgq_pop();
    }
  else if (motor_interface->msgq_first_is<MotorInterface::SubscribeMessage>() )
    {
      MotorInterface::SubscribeMessage* msg = motor_interface->msgq_first<MotorInterface::SubscribeMessage>();
        
      logger->log_info("MotorThread", "++++++++++++++++++++++++++++++++++++++++++++++Subscriber %lu", msg->getSubscriber());
     
     
      motor_interface->setControllerID(msg->getSubscriber());
      motor_interface->write();
      
      motor_interface->msgq_pop();
    }
  else
    {
      if(motor_interface->msgq_size() > 0)
        {
          logger->log_error("MotorThread", "Message of invalid type received from %s", motor_interface->msgq_first()->sender());
 
          motor_interface->msgq_pop();
        }
    }
  /*   
       if((++logger_modulo_counter %= 100) == 0)
       {
       logger->log_info("MotorThread", "3 forward command: %f  sideward command: %f  rotation command: %f speed commdan: %f", 
       forward,
       sideward,
       rotation,
       speed);
                                                                                                                  
       }
  */   
       
  double dir = 60;
  double alpha          = ((2./3.) * ((cos(deg2rad(dir))                * sideward)     + (sin(deg2rad(dir))                    * forward))) * speed    + rotation;
  double beta           = ((2./3.) * ((cos(deg2rad(dir + 120))  * sideward)     + (sin(deg2rad(dir + 120))      * forward)))  * speed  + rotation;
  double gamma          = ((2./3.) * ((cos(deg2rad(dir + 240))  * sideward)     + (sin(deg2rad(dir + 240))      * forward)))  * speed  + rotation;
           
#ifndef NO_VMC 
  if(alpha != 0 || beta != 0 || gamma != 0)
    // && (++timeout_counter < 1000))
    {  
      apiObject->useVMC().MotorRPMs.Set(alpha, beta, gamma);
      timeout_counter = 0;
      // logger->log_error("MotorThread", "run");
    }
  else
    {
      apiObject->useVMC().MotorRPMs.Set(0, 0, 0);
      timeout_counter = 0;
      // logger->log_warn("MotorThread", "STOP");

    }
  usleep(15000);
  /* else
     {
     timeout_counter = 2000;
     logger->log_error("MotorThread", "STOP");
      
     if(alpha != 0 || beta != 0 || gamma != 0)
     {
     timeout_counter = 0;
     }
     }*/
#endif
  if ( (alpha != old_alpha) || (beta != old_beta) || (gamma != old_gamma) ) {
    logger->log_info("MotorThread", " alpha : %f, beta: %f, gamma: %f, rotation: %f", alpha, beta, gamma, rotation);
    old_alpha = alpha;
    old_beta  = beta;
    old_gamma = gamma;
  }

           
  /*
    logger->log_info("MotorThread", "RPM1: %f  RPM2: %f  RPM3: %f ", 
    apiObject->useVMC().Motor[0].ActualRPM.getValue(),
    apiObject->useVMC().Motor[1].ActualRPM.getValue(),
    apiObject->useVMC().Motor[2].ActualRPM.getValue());
  */
  /*
    motor_interface->setRPM1((int)apiObject->useVMC().Motor[0].ActualRPM.getValue());
    motor_interface->setRPM2((int)apiObject->useVMC().Motor[1].ActualRPM.getValue());
    motor_interface->setRPM3((int)apiObject->useVMC().Motor[2].ActualRPM.getValue());
  */
        
  /*
    motor_interface->setRPM1((int)forward++);
    motor_interface->setRPM2((int)sideward++);
    motor_interface->setRPM3((int)rotation++);
    motor_interface->write();
  */
        
}
