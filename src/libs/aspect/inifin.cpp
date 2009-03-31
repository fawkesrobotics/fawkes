
/***************************************************************************
 *  inifin.h - Fawkes Aspect initializer/finalizer
 *
 *  Created: Tue Jan 30 13:36:42 2007
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include <aspect/inifin.h>

#include <core/threading/thread.h>
#include <core/macros.h>
#include <blackboard/blackboard.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/clock.h>
#include <aspect/fawkes_network.h>
#include <aspect/network.h>
#include <aspect/thread_producer.h>
#include <aspect/time_source.h>
#include <aspect/mainloop.h>
#include <aspect/mainloop/employer.h>
#include <aspect/logger.h>
#include <aspect/logger/employer.h>
#include <aspect/plugin_director.h>
#ifdef HAVE_FIREVISION
#include <aspect/vision_master.h>
#include <aspect/vision.h>
#endif

namespace fawkes {

/** @class AspectIniFin <aspect/inifin.h>
 * Fawkes Aspect Initializer/Finalizer.
 * Initializes certain thread aspects.
 * All aspects defined in the Fawkes tree are supported and properly
 * initialized such that guarantees are met.
 * @see Aspects
 * @author Tim Niemueller
 */


/** Constructor.
 * @param blackboard BlackBoard
 * @param collector Thread collector
 * @param config Configuration
 * @param logger Logger
 * @param clock Clock
 */
AspectIniFin::AspectIniFin(BlackBoard *blackboard,
			   ThreadCollector *collector,
			   Configuration *config,
			   Logger *logger,
			   Clock *clock)

{
  __blackboard        = blackboard;
  __thread_collector  = collector;
  __config            = config;
  __logger            = logger;
  __clock             = clock;
  __fnethub           = NULL;
  __nnresolver        = NULL;
  __service_publisher = NULL;
  __service_browser   = NULL;
  __mainloop_employer = NULL;
}


/** Destructor. */
AspectIniFin::~AspectIniFin()
{
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
  __fnethub = fnethub;
}


/** Set Fawkes MainLoopEmployer.
 * Use this to set the Fawkes main loop employer. The main loop employer is used
 * to set a new main loop of a plugin with the MainLoopAspect. Uniqueness is
 * quaranteed such that only a single main loop exists at any given time.
 * @param employer main loop employer
 */
void
AspectIniFin::set_mainloop_employer(MainLoopEmployer *employer)
{
  __mainloop_employer = employer;
}


/** Set Fawkes LoggerEmployer.
 * @param employer logger employer
 */
void
AspectIniFin::set_logger_employer(LoggerEmployer *employer)
{
  __logger_employer = employer;
}


/** Set Fawkes BlockedTimingExecutor.
 * Use this to set the Fawkes blocked timing executor.
 * @param btexec blocked timing executor instance
 */
void
AspectIniFin::set_blocked_timing_executor(BlockedTimingExecutor *btexec)
{
  __btexec = btexec;
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
  __nnresolver = nnresolver;
  __service_publisher = service_publisher;
  __service_browser = service_browser;
}


/** Set plugin manager.
 * @param manager PluginManager instance
 */
void
AspectIniFin::set_plugin_manager(PluginManager *manager)
{
  __plugin_manager = manager;
}


/** Initialize thread.
 * @param thread thread to initialize
 */
void
AspectIniFin::init(Thread *thread)
{
  // printf("Initializing thread %s\n", thread->name());

  BlockedTimingAspect *blocked_timing_thread __unused;
  if ( (blocked_timing_thread = dynamic_cast<BlockedTimingAspect *>(thread)) != NULL ) {
    if ( thread->opmode() != Thread::OPMODE_WAITFORWAKEUP ) {
      throw CannotInitializeThreadException("Thread '%s' not in WAITFORWAKEUP mode "
					    "(required for BlockedTimingAspect)",
					    thread->name());
    }
  }

  BlackBoardAspect *blackboard_thread;
  if ( (blackboard_thread = dynamic_cast<BlackBoardAspect *>(thread)) != NULL ) {
    blackboard_thread->init_BlackBoardAspect( __blackboard );
  }

  ThreadProducerAspect *thread_producer_thread;
  if ( (thread_producer_thread = dynamic_cast<ThreadProducerAspect *>(thread)) != NULL ) {
    thread_producer_thread->init_ThreadProducerAspect( __thread_collector );
  }

  ConfigurableAspect *configurable_thread;
  if ( (configurable_thread = dynamic_cast<ConfigurableAspect *>(thread)) != NULL ) {
    configurable_thread->init_ConfigurableAspect(__config);
  }

  LoggingAspect *logging_thread;
  if ( (logging_thread = dynamic_cast<LoggingAspect *>(thread)) != NULL ) {
    logging_thread->init_LoggingAspect(__logger);
  }

  ClockAspect *clock_thread;
  if ( (clock_thread = dynamic_cast<ClockAspect *>(thread)) != NULL ) {
    clock_thread->init_ClockAspect(__clock);
  }

  PluginDirectorAspect *plugin_director_thread;
  if ( (plugin_director_thread = dynamic_cast<PluginDirectorAspect *>(thread)) != NULL ) {
    if ( thread->opmode() != Thread::OPMODE_CONTINUOUS ) {
      throw CannotInitializeThreadException("Thread '%s' not in CONTINUOUS mode "
					    "(required for PluginDirectorAspect)",
					    thread->name());
    }
    if ( __plugin_manager ) {
      plugin_director_thread->init_PluginDirectorAspect(__plugin_manager);
    } else {
      throw CannotInitializeThreadException("Thread '%s' has PluginDirectorAspect "
					    "but no PluginManager has been set in "
					    "AspectIniFin", thread->name());
    }
  }

  FawkesNetworkAspect *fnet_thread;
  if ( (fnet_thread = dynamic_cast<FawkesNetworkAspect *>(thread)) != NULL ) {
    if ( __fnethub == NULL ) {
      throw CannotInitializeThreadException("Thread '%s' has FawkesNetworkAspect but no "
					    "FawkesNetworkHub has been set in AspectIniFin",
					    thread->name());
    }
    fnet_thread->init_FawkesNetworkAspect(__fnethub);
  }

#ifdef HAVE_FIREVISION
  VisionMasterAspect *vision_master_thread;
  if ( (vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread)) != NULL ) {
    try {
      __vision_dependency.add(vision_master_thread);
      thread->add_notification_listener(this);
    } catch (DependencyViolationException &e) {
      CannotInitializeThreadException ce("Dependency violation for VisionProviderAspect "
					 "detected");
      ce.append(e);
      throw ce;
    }
  }

  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    try {
      if ( (vision_thread->vision_thread_mode() == VisionAspect::CONTINUOUS) &&
	   (thread->opmode() != Thread::OPMODE_CONTINUOUS) ) {
	throw CannotInitializeThreadException("Vision thread '%s' operates in continuous "
					      "mode but thread does not", thread->name());
      }
      if ( (vision_thread->vision_thread_mode() == VisionAspect::CYCLIC) &&
	   (thread->opmode() != Thread::OPMODE_WAITFORWAKEUP) ) {
	throw CannotInitializeThreadException("Vision thread '%s' operates in cyclic mode but"
					      "thread does not operate in wait-for-wakeup "
					      "mode.", thread->name());
      }
      __vision_dependency.add(vision_thread);
      vision_thread->init_VisionAspect( __vision_dependency.provider()->vision_master() );
      thread->add_notification_listener(this);
    } catch (DependencyViolationException &e) {
      CannotInitializeThreadException ce("Dependency violation for VisionAspect detected");
      ce.append(e);
      throw ce;
    }
  }
#endif /* HAVE_FIREVISION */

  NetworkAspect *net_thread;
  if ( (net_thread = dynamic_cast<NetworkAspect *>(thread)) != NULL ) {
    if ( (__nnresolver == NULL) || (__service_publisher == NULL) ||
	 (__service_browser == NULL) ) {
      throw CannotInitializeThreadException("Thread has NetworkAspect but required data "
					    "has not been set in AspectIniFin");
    }
    net_thread->init_NetworkAspect(__nnresolver, __service_publisher, __service_browser);
  }

  TimeSourceAspect *timesource_thread;
  if ( (timesource_thread = dynamic_cast<TimeSourceAspect *>(thread)) != NULL ) {
    try {
      __timesource_uc.add(timesource_thread->get_timesource());
      __clock->register_ext_timesource(timesource_thread->get_timesource(),
				       /* make default */ true);
    } catch (...) {
      throw CannotInitializeThreadException("Thread has TimeSourceAspect but there is "
					    "already another time provider.");
    }
  }

  MainLoopAspect *mainloop_thread;
  if ( (mainloop_thread = dynamic_cast<MainLoopAspect *>(thread)) != NULL ) {
    if (thread->opmode() != Thread::OPMODE_WAITFORWAKEUP) {
      throw CannotInitializeThreadException("MainLoopAspect thread must operate "
					    "in wait-for-wakeup mode.");
      
    }
    if ( __mainloop_employer == NULL ) {
      throw CannotInitializeThreadException("Thread has MainLoopAspect but no "
					    "MainLoopEmployer has been set.");
    }
    if ( __btexec == NULL ) {
      throw CannotInitializeThreadException("Thread has MainLoopAspect but no "
					    "BlockedTimingExecutor has been set.");
    }
    try {
      __mainloop_uc.add(mainloop_thread);
      mainloop_thread->init_MainLoopAspect(__btexec);
      thread->add_notification_listener(this);
    } catch (Exception &e) {
      CannotInitializeThreadException ce("Main loop thread failed to initialize");
      ce.append(e);
      throw ce;
    }
  }

  LoggerAspect *logger_thread;
  if ( (logger_thread = dynamic_cast<LoggerAspect *>(thread)) != NULL ) {
    if ( __logger_employer == NULL ) {
      throw CannotInitializeThreadException("Thread has LoggerAspect but no "
					    "LoggerEmployer has been set.");
    }
    try {
      __logger_employer->add_logger(logger_thread->get_logger());
    } catch (Exception &e) {
      CannotInitializeThreadException ce("Thread has LoggerAspect but Logger "
					 "could not be added.");
      ce.append(e);
      throw ce;
    } catch (...) {
      throw CannotInitializeThreadException("Thread has LoggerAspect but Logger "
					    "could not be added.");
    }
  }
}


bool
AspectIniFin::prepare_finalize(Thread *thread)
{
#ifdef HAVE_FIREVISION
  VisionMasterAspect *vision_master_thread;
  if ( (vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread)) != NULL ) {
    if ( ! __vision_dependency.can_remove(vision_master_thread) ) {
      __logger->log_error("AspectIniFin", "Cannot remove vision master, there are "
			  "still vision threads that depend on it");
      return false;
    }
  }

  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    if ( ! __vision_dependency.can_remove(vision_thread) ) {
      __logger->log_error("AspectIniFin", "Cannot remove vision thread, dependency "
			  "violation");
      return false;
    }
  }
#endif /* HAVE_FIREVISION */

  return true;
}


/** Finalize thread.
 * @param thread thread to finalize
 */
void
AspectIniFin::finalize(Thread *thread)
{
#ifdef HAVE_FIREVISION
  VisionMasterAspect *vision_master_thread;
  if ( (vision_master_thread = dynamic_cast<VisionMasterAspect *>(thread)) != NULL ) {
    try {
      __vision_dependency.remove(vision_master_thread);
    } catch (DependencyViolationException &e) {
      CannotFinalizeThreadException ce("Dependency violation for VisionProviderAspect "
				       "detected");
      ce.append(e);
      throw ce;
    }
  }

  VisionAspect *vision_thread;
  if ( (vision_thread = dynamic_cast<VisionAspect *>(thread)) != NULL ) {
    __vision_dependency.remove(vision_thread);
  }
#endif /* HAVE_FIREVISION */

  TimeSourceAspect *timesource_thread;
  if ( (timesource_thread = dynamic_cast<TimeSourceAspect *>(thread)) != NULL ) {
    try {
      __clock->remove_ext_timesource(timesource_thread->get_timesource());
      __timesource_uc.remove(timesource_thread->get_timesource());
    } catch (Exception &e) {
      CannotFinalizeThreadException ce("Failed to remove time source");
      ce.append(e);
      throw;
    }
  }

  MainLoopAspect *mainloop_thread;
  if ( (mainloop_thread = dynamic_cast<MainLoopAspect *>(thread)) != NULL ) {
    try {
      if (__mainloop_uc.resource() == mainloop_thread) {
	__mainloop_employer->set_mainloop_thread(NULL);
	__mainloop_uc.remove(mainloop_thread);
      }
    } catch (Exception &e) {
      CannotFinalizeThreadException ce("Failed to remove main loop");
      ce.append(e);
      throw;
    }
  }

  LoggerAspect *logger_thread;
  if ( (logger_thread = dynamic_cast<LoggerAspect *>(thread)) != NULL ) {
    try {
      __logger_employer->remove_logger(logger_thread->get_logger());
    } catch (Exception &e) {
      CannotFinalizeThreadException ce("Failed to remove logger");
      ce.append(e);
      throw;
    }
  }
}


bool
AspectIniFin::thread_started(Thread *thread) throw()
{
  MainLoopAspect *mainloop_thread;
  if ( (mainloop_thread = dynamic_cast<MainLoopAspect *>(thread)) != NULL ) {
    try {
      __mainloop_employer->set_mainloop_thread(thread);
    } catch (Exception &e) {
      __logger->log_error("AspectIniFin", "Main loop thread started successfully but "
			  "could not add main loop thread's main loop");
    }
  }

  return false;
}


bool
AspectIniFin::thread_init_failed(Thread *thread) throw()
{
  MainLoopAspect *mainloop_thread;
  if ( (mainloop_thread = dynamic_cast<MainLoopAspect *>(thread)) != NULL ) {
    try {
      __mainloop_uc.remove(mainloop_thread);
    } catch (Exception &e) {
      __logger->log_error("AspectIniFin", "Failed to remove main loop from uniqueness "
			  "constraint on thread init fail of %s", thread->name());
    }
  }

  try {
    finalize(thread);
  } catch (Exception &e) {
    __logger->log_error("AspectIniFin", "Initialization of thread '%s' failed, but "
			"the thread thread could not be internally finalized",
			thread->name());
    __logger->log_error("AspectIniFin", e);
  }

  return false;
}

} // end namespace fawkes
