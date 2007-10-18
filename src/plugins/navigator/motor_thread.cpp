
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
#include <interfaces/motor.h>
#include <utils/math/angle.h>

#include <vmc/LayerClasses/CvmcAPI.h>
#include <vmc/SupportClasses/Enums.h>

#include <cmath>
#include <unistd.h>

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
 
/** Constructor. */
MotorThread::MotorThread()
  : Thread("MotorThread", Thread::OPMODE_CONTINUOUS)
{
  motor_interface = NULL;
  forward = 0;
  sideward = 0;
  rotation = 0;
  speed = 0;
  logger_modulo_counter = 0;
  timeout_counter = 0;
  old_alpha = old_beta = old_gamma = 0.f;
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

  apiObject = new VMC::CvmcAPI();

  apiObject->selectHardwareAdapter(VMC::RS232);
  if ( ! apiObject->selectDevice("/dev/ttyS0") ) {
    throw Exception("MotorThread failed to open serial device VMC");
  }

  motor_interface->setMotorState(MotorInterface::MOTOR_ENABLED);
  motor_interface->write();
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
    
  apiObject->closeDevice();
  delete apiObject;
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
  if ( ! motor_interface->msgq_empty() ) {
    if ( motor_interface->msgq_first_is<MotorInterface::TransRotRPMMessage>() )
      {
	MotorInterface::TransRotRPMMessage* msg = motor_interface->msgq_first<MotorInterface::TransRotRPMMessage>();

	if ( msg->sender_id() == motor_interface->getControllerThreadID() ) {
	  forward = msg->getForward();
	  sideward = msg->getSideward();
	  rotation = msg->getRotation() / 3; // divide by three (motors)
	  speed = msg->getSpeed();
          /*
	  logger->log_debug(name(), "Processing TransRotRPMMessage, forward: %f, "
			            "sideward: %f, rotation: %f, speed: %f",
			    forward, sideward, rotation, speed);
          */
	} else {
	  logger->log_warn(name(), "Warning, received TransRotRPMMessage of thread %s (%lu), "
			   "but the motor is currently controlled by thread %s (%lu)",
			   msg->sender(), msg->sender_id(),
			   motor_interface->getControllerThreadName(),
			   motor_interface->getControllerThreadID());
	}
      }
    else if (motor_interface->msgq_first_is<MotorInterface::AquireControlMessage>() )
      {
	MotorInterface::AquireControlMessage* msg = motor_interface->msgq_first<MotorInterface::AquireControlMessage>();

	if ( msg->getThreadID() == 0 ) {
	  motor_interface->setControllerThreadID(msg->sender_id());
	  motor_interface->setControllerThreadName(msg->sender());
	} else {
	  motor_interface->setControllerThreadID(msg->getThreadID());
	  motor_interface->setControllerThreadName(msg->getThreadName());
	}
	motor_interface->write();

	logger->log_debug(name(), "Thread %s (%lu) aquired motor control",
			  motor_interface->getControllerThreadName(),
			  motor_interface->getControllerThreadID());
      }
    else if (motor_interface->msgq_first_is<MotorInterface::SetMotorStateMessage>() )
      {
	MotorInterface::SetMotorStateMessage* msg = motor_interface->msgq_first<MotorInterface::SetMotorStateMessage>();
	// we really want to make sure that we got a correct message with useful values
	// thus we check every single value
	if ( msg->getMotorState() == MotorInterface::MOTOR_ENABLED ) {
	  motor_interface->setMotorState(MotorInterface::MOTOR_ENABLED);
          logger->log_info(name(), "Enabling motor control");
	} else if ( msg->getMotorState() == MotorInterface::MOTOR_DISABLED ) {
	  motor_interface->setMotorState(MotorInterface::MOTOR_DISABLED);
          logger->log_info(name(), "Disabling motor control");
	} else {
	  logger->log_error(name(), "SetMotorStateMessage received with illegal value: %u",
			    msg->getMotorState());
	}
      }
    else
      {
	logger->log_error("MotorThread", "Message of invalid type received from %s", motor_interface->msgq_first()->sender());
      }

    motor_interface->msgq_pop();
  }

  double dir    = 60;
  double alpha  = ((2./3.) * ((cos(deg2rad(dir))       * sideward) + (sin(deg2rad(dir))       * forward))) * speed  + rotation;
  double beta   = ((2./3.) * ((cos(deg2rad(dir + 120)) * sideward) + (sin(deg2rad(dir + 120)) * forward))) * speed  + rotation;
  double gamma  = ((2./3.) * ((cos(deg2rad(dir + 240)) * sideward) + (sin(deg2rad(dir + 240)) * forward))) * speed  + rotation;

  /* VERY (!) noisy, use only for debugging
  logger->log_info("MotorThread", " alpha : %f, beta: %f, gamma: %f, rotation: %f"
		   " old_alpha : %f, old_beta: %f, old_gamma: %f",
		   alpha, beta, gamma, rotation,
		   old_alpha, old_beta, old_gamma );
  */
           
  if ( motor_interface->getMotorState() == MotorInterface::MOTOR_ENABLED ) {
    if(alpha != 0 || beta != 0 || gamma != 0) {  
	apiObject->useVMC().MotorRPMs.Set(alpha, beta, gamma);
	timeout_counter = 0;
    } else {
      apiObject->useVMC().MotorRPMs.Set(0, 0, 0);
      timeout_counter = 0;
    }
    usleep(15000);
  }
  if ( (alpha != old_alpha) || (beta != old_beta) || (gamma != old_gamma) ) {
    logger->log_info("MotorThread", " alpha : %f, beta: %f, gamma: %f, rotation: %f",
		     alpha, beta, gamma, rotation);
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
