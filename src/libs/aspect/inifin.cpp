
/***************************************************************************
 *  aspect_initializer.h - Fawkes Aspect initializer
 *
 *  Created: Tue Jan 30 13:36:42 2007
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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
 *  along with this program; if not, write to the Free Software Foundation,
 *  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.
 */

#include <aspect/inifin.h>

#include <core/threading/thread.h>
#include <blackboard/blackboard.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/fawkes_network.h>
#include <aspect/vision_master.h>
#include <aspect/vision.h>
#include <aspect/network.h>

#include <utils/constraints/dependency_onetomany.h>

/** @class AspectIniFin aspect/inifin.h
 * Fawkes Aspect Initializer/Finalizer.
 * Initializes certain thread aspects.
 * All aspects defined in the Fawkes tree are supported and properly
 * initialized such that guarantees are met.
 * @see Aspects
 * @author Tim Niemueller
 */


/** Constructor.
 * @param blackboard BlackBoard
 * @param config Configuration
 * @param logger Logger
 * @param clock Clock
 */
AspectIniFin::AspectIniFin(BlackBoard *blackboard,
			   Configuration *config,
			   Logger *logger,
			   Clock *clock)

{
  this->blackboard = blackboard;
  this->config     = config;
  this->logger     = logger;
  this->clock      = clock;
  this->fnethub    = NULL;
  this->nnresolver = NULL;
  this->service_publisher = NULL;
  this->service_browser = NULL;

  vision_dependency = new OneToManyDependency<VisionMasterAspect, VisionAspect>();
}


/** Destructor. */
AspectIniFin::~AspectIniFin()
{
  delete vision_dependency;
}


/** Set Fawkes Network Hub.
 * Use this to set the Fawkes Network Hub. If you do not use the Fawkes Network
 * you do not need to call this function to set a hub. In that case threads that
 * demand the hub will cause an exception to be thrown that the thread cannot be
 * initialized.
 * @param fnethub Fawkes Network Hub
 */
void
AspectIniFin::set_fnet_hub(FawkesNetworkHub *fnethub)
{
  this->fnethub = fnethub;
}


/** Set Fawkes Network Hub.
 * Use this to initialize the NetworkAspect. If you do not use the Network Aspect
 * you do not need to call this function to set a hub. In that case threads that
 * demand this aspect will cause an exception to be thrown that the thread cannot be
 * initialized.
 * @param nnresolver network name resolver
 * @param service_publisher service publisher
 * @param service_browser service browser
 */
void
AspectIniFin::set_network_members(NetworkNameResolver *nnresolver,
				  ServicePublisher *service_publisher,
				  ServiceBrowser *service_browser)
{
  this->nnresolver = nnresolver;
  this->service_publisher = service_publisher;
  this->service_browser = service_browser;
}


/** Initialize thread.
 * @param thread thread to initialize
 */
void
AspectIniFin::init(Thread *thread)
{
  // printf("Initializing thread %s\n", thread->name());

  BlockedTimingAspect *blocked_timing_thread;
  if ( (blocked_timing_thread = dynamic_cast<BlockedTimingAspect *>(thread)) != NULL ) {
    if ( thread->opmode() != Thread::OPMODE_WAITFORWAKEUP ) {
      throw CannotInitializeThreadException("Thread not in WAITFORWAKEUP mode (required for BlockedTimingAspect)");
    }
  }

  BlackBoardAspect *blackboard_thread;
  if ( (blackboard_thread = dynamic_cast<BlackBoardAspect *>(thread)) != NULL ) {
    blackboard_thread->initBlackBoardAspect( blackboard->interface_manager() );
  }

  ConfigurableAspect *configurable_thread;
  if ( (configurable_thread = dynamic_cast<ConfigurableAspect *>(thread)) != NULL ) {
    configurable_thread->initConfigurableAspect(config);
  }

  LoggingAspect *logging_thread;
  if ( (logging_thread = dynamic_cast<LoggingAspect *>(thread)) != NULL ) {
    logging_thread->initLoggingAspect(logger);
  }

  ClockAspect *clock_thread;
  if ( (clock_thread = dynamic_cast<ClockAspect *>(thread)) != NULL ) {
    clock_thread->initClockAspect(clock);
  }

  FawkesNetworkAspect *fnet_thread;
  if ( (fnet_thread = dynamic_cast<FawkesNetworkAspect *>(thread)) != NULL ) {
    if ( fnethub == NULL ) {
      throw CannotInitializeThreadException("Thread has FawkesNetworkAspect but no FawkesNetworkHub has been set in AspectIniFin");
    }
    fnet_thread->initFawkesNetworkAspect(fnethub);
  }

  VisionMasterAspect *vision_master_thread;
  if ( (vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread)) != NULL ) {
    try {
      vision_dependency->add(vision_master_thread);
    } catch (DependencyViolationException &e) {
      CannotInitializeThreadException ce("Dependency violation for VisionProviderAspect detected");
      ce.append(e);
      throw ce;
    }
  }

  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    try {
      if ( (vision_thread->vision_thread_mode() == VisionAspect::CONTINUOUS) &&
	   (thread->opmode() != Thread::OPMODE_CONTINUOUS) ) {
	throw CannotInitializeThreadException("Vision thread operates in continuous "
					      "mode but thread does not");
      }
      if ( (vision_thread->vision_thread_mode() == VisionAspect::CYCLIC) &&
	   (thread->opmode() != Thread::OPMODE_WAITFORWAKEUP) ) {
	throw CannotInitializeThreadException("Vision thread operates in cyclic mode but"
					      "thread does not operate in wait-for-wakeup "
					      "mode.");
      }
      vision_dependency->add(vision_thread);
      vision_thread->initVisionAspect( vision_dependency->provider()->vision_master() );
    } catch (DependencyViolationException &e) {
      CannotInitializeThreadException ce("Dependency violation for VisionAspect detected");
      ce.append(e);
      throw ce;
    }
  }

  NetworkAspect *net_thread;
  if ( (net_thread = dynamic_cast<NetworkAspect *>(thread)) != NULL ) {
    if ( (nnresolver == NULL) || (service_publisher == NULL) || (service_browser == NULL) ) {
      throw CannotInitializeThreadException("Thread has NetworkAspect but required data has not been set in AspectIniFin");
    }
    net_thread->initNetworkAspect(nnresolver, service_publisher, service_browser);
  }

}


bool
AspectIniFin::prepare_finalize(Thread *thread)
{
  VisionMasterAspect *vision_master_thread;
  if ( (vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread)) != NULL ) {
    if ( ! vision_dependency->can_remove(vision_master_thread) ) {
      return false;
    }
  }

  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    if ( ! vision_dependency->can_remove(vision_thread) ) {
      return false;
    }
  }

  return true;
}


/** Finalize thread.
 * @param thread thread to finalize
 */
void
AspectIniFin::finalize(Thread *thread)
{
  VisionMasterAspect *vision_master_thread;
  if ( (vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread)) != NULL ) {
    try {
      vision_dependency->remove(vision_master_thread);
    } catch (DependencyViolationException &e) {
      CannotFinalizeThreadException ce("Dependency violation for VisionProviderAspect detected");
      ce.append(e);
      throw ce;
    }
  }

  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    vision_dependency->remove(vision_thread);
  }
}
