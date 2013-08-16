
/***************************************************************************
 *  context_thread.cpp - OpenNI context providing Thread
 *
 *  Created: Sat Feb 26 17:46:29 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
 *
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

#include "context_thread.h"
#include "utils/version.h"

#include <cerrno>
#include <csignal>
#include <unistd.h>
#include <sys/wait.h>
#include <XnCppWrapper.h>

using namespace fawkes;

/** @class OpenNiContextThread "context_thread.h"
 * OpenNI Context Thread.
 * This thread maintains an OpenNI context which can be used by other
 * threads and is provided via the OpenNiAspect.
 *
 * @author Tim Niemueller
 */

/** Constructor. */
OpenNiContextThread::OpenNiContextThread()
  : Thread("OpenNiContextThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE),
    AspectProviderAspect(&__openni_aspect_inifin)
{
}


/** Destructor. */
OpenNiContextThread::~OpenNiContextThread()
{
}


void
OpenNiContextThread::init()
{
  __sensor_server_pid = -1;
  __cfg_run_sensor_server = false;
  try {
    __cfg_run_sensor_server =
      config->get_bool("/plugins/openni/run_sensor_server");
  } catch (Exception &e) {} // ignore and use default
  if (__cfg_run_sensor_server) {
    __cfg_sensor_bin = config->get_string("/plugins/openni/sensor_server_bin");
  }

  __openni = new xn::Context();

  XnStatus st;
  if ((st = __openni->Init()) != XN_STATUS_OK) {
    __openni.clear();
    throw Exception("Initializing OpenNI failed: %s", xnGetStatusString(st));
  }

  __last_refcount = __openni.refcount();

  __check_now.set_clock(clock);
  __check_last.set_clock(clock);
  __check_last.stamp();

  __device_no_data_loops = 0;
  __openni_aspect_inifin.set_openni_context(__openni);

  if (__cfg_run_sensor_server) {
    start_sensor_server();

    // We don't want the server to die, we kill it by ourself.
    // Therefore we hold an instance all the time. Since client
    // connections are not properly terminated on unloading openni,
    // setting the timeout to 0 to stop the server immediately after
    // it is no longer used does not work.
    xn::NodeInfoList list;
    if (__openni->EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, list)
	== XN_STATUS_OK)
    {
      for (xn::NodeInfoList::Iterator i = list.Begin(); i != list.End(); ++i) {
	if ((*i).GetDescription().Type == XN_NODE_TYPE_DEVICE) {
	  __device = new xn::Device();
	  (*i).GetInstance(*__device);
	  break;
	}
      }
    }
  }
}


void
OpenNiContextThread::finalize()
{
  __openni->StopGeneratingAll();

#if XN_VERSION_GE(1,3,2,0)
  __openni->Release();
#else
  __openni->Shutdown();
#endif
  __openni.clear();
  __openni_aspect_inifin.set_openni_context(__openni);

  if (__cfg_run_sensor_server) {
    delete __device;
    stop_sensor_server();
  }
}


void
OpenNiContextThread::loop()
{
  __openni.lock();
  if (__openni.refcount() != __last_refcount) {
    print_nodes();
    __last_refcount = __openni.refcount();
  }
  __openni->WaitNoneUpdateAll();

  __check_now.stamp();
  if ((__check_now - &__check_last) > 5) {
    verify_active();
    __check_last = __check_now;
  }

  __openni.unlock();
}

inline const char *
type_to_string(XnProductionNodeType type)
{
  switch (type) {
  case XN_NODE_TYPE_DEVICE:   return "device";
  case XN_NODE_TYPE_DEPTH:    return "depth";
  case XN_NODE_TYPE_IMAGE:    return "image";
  case XN_NODE_TYPE_AUDIO:    return "audio";
  case XN_NODE_TYPE_IR:       return "IR";
  case XN_NODE_TYPE_USER:     return "user";
  case XN_NODE_TYPE_RECORDER: return "recorder";
  case XN_NODE_TYPE_PLAYER:   return "player";
  case XN_NODE_TYPE_GESTURE:  return "gesture";
  case XN_NODE_TYPE_SCENE:    return "scene";
  case XN_NODE_TYPE_HANDS:    return "hands";
  case XN_NODE_TYPE_CODEC:    return "codec";
  default:                    return "unknown";
  }
}

/** Print active nodes to log.
 * Assumes that the context has been locked.
 */
void
OpenNiContextThread::print_nodes()
{
  xn::NodeInfoList nodes;
  if (__openni->EnumerateExistingNodes(nodes) == XN_STATUS_OK) {
    logger->log_info(name(), "Currently existing nodes:");
    for (xn::NodeInfoList::Iterator n = nodes.Begin(); n != nodes.End(); ++n) {
      const XnProductionNodeDescription &pnd = (*n).GetDescription();
      const char *info = (*n).GetCreationInfo();
      if (strlen(info) == 0)  info = NULL;

      xn::Generator generator;
      bool have_gen = ((*n).GetInstance(generator) == XN_STATUS_OK);

      logger->log_info(name(), "  %-8s %8s (type: %-8s  vendor: %-12s  name: %-24s  "
		       "version: %u.%u.%u.%u%s%s)",
		       (*n).GetInstanceName(),
		       have_gen ? (generator.IsGenerating() ? "active" : "inactive") : "unknown",
		       type_to_string(pnd.Type), pnd.strVendor, pnd.strName,
		       pnd.Version.nMajor, pnd.Version.nMinor, pnd.Version.nMaintenance,
		       pnd.Version.nBuild, info ? "  info: " : "", info ? info : "");
    }
  }
}


/** Verify that all nodes are active.
 * Assumes that the context has been locked.
 */
void
OpenNiContextThread::verify_active()
{
  xn::NodeInfoList nodes;
  if (__openni->EnumerateExistingNodes(nodes) == XN_STATUS_OK) {
    for (xn::NodeInfoList::Iterator n = nodes.Begin(); n != nodes.End(); ++n) {
      xn::Generator generator;
      bool have_gen = ((*n).GetInstance(generator) == XN_STATUS_OK);

      if (have_gen) {
	const XnProductionNodeDescription &pnd = (*n).GetDescription();
	// do not verify on device nodes for now, always reports inactive :-/
        if (pnd.Type != XN_NODE_TYPE_DEVICE) {
          if (! generator.IsGenerating()) {
            logger->log_warn(name(), "Inactive node '%s' (%s, %s/%s), trying to activate",
                             (*n).GetInstanceName(), type_to_string(pnd.Type),
                             pnd.strVendor, pnd.strName);
            generator.StartGenerating();

          } else if (! generator.IsDataNew()) {
            if (__dead_loops.find((*n).GetInstanceName()) != __dead_loops.end()) {
              __dead_loops[(*n).GetInstanceName()] += 1;
            } else {
              __dead_loops[(*n).GetInstanceName()]  = 1;
            }

          } else if (__dead_loops.find((*n).GetInstanceName()) != __dead_loops.end()) {
            __dead_loops.erase((*n).GetInstanceName());
          }

          /* The following does not work atm because IsDataNew() always reports false.
             While this could be because the WaitNoneUpdateAll() did not yet update the
             device node, event the timestamp does not change, therefore rendering this
             way to detect death of a device node unusable.

	} else if (pnd.Type == XN_NODE_TYPE_DEVICE) {
	  // as an alternative, verify how often it has not been updated
	  if (generator.IsDataNew()) {
	    __device_no_data_loops = 0;
	  } else {
	    if (++__device_no_data_loops > 10) {
	      logger->log_warn(name(), "Device '%s' had no fresh data for long time. "
			       "Reload maybe necessary.", (*n).GetInstanceName());
	    }
	  }
          */
	}

	xn::ErrorStateCapability ecap = generator.GetErrorStateCap();
	if (ecap.GetErrorState() != XN_STATUS_OK) {
	  logger->log_warn(name(), "ERROR in node '%s': %s", (*n).GetInstanceName(),
			   xnGetStatusString(ecap.GetErrorState()));
	}
      }
    }
  }

  std::map<std::string, unsigned int>::iterator d;
  for (d = __dead_loops.begin(); d != __dead_loops.end(); ++d) {
    if (d->second >= 3) {
      logger->log_warn(name(), "Node '%s' had no fresh data for long time (%u tests)",
                       d->first.c_str(), d->second);
    }
  }
}

/** Start the sensor server daemon. */
void
OpenNiContextThread::start_sensor_server()
{
  if (__sensor_server_pid != -1) {
    throw Exception("Sensor server appears to be already running");
  }

  logger->log_info(name(), "Starting XnSensorServer");

  pid_t pid = fork();
  if (pid == -1) {
    throw Exception(errno, "Forking for new process failed: %s");
  } else if (pid == 0) {
    // child
    setsid();
    // ignore SIGINT, we will propagate it as SIGTERM on unload
    signal(SIGINT, SIG_IGN);
    fclose(stdout);
    fclose(stdin);
    fclose(stderr);
    char *argv[] = {(char *)__cfg_sensor_bin.c_str(), NULL};
    if (execve(__cfg_sensor_bin.c_str(), argv, environ) == -1) {
      throw Exception("Failed to execute %s, exited with %i: %s\n",
		      __cfg_sensor_bin.c_str(), errno, strerror(errno));
    }
  }

  __sensor_server_pid = pid;
}

void
OpenNiContextThread::stop_sensor_server()
{
  if (__sensor_server_pid == -1) {
    throw Exception("Sensor server appears not to be already running");
  }

  logger->log_info(name(), "Stopping XnSensorServer");
  ::kill(__sensor_server_pid, SIGTERM);
  for (unsigned int i = 0; i < 200; ++i) {
    usleep(10000);
    int status;
    int rv = waitpid(__sensor_server_pid, &status, WNOHANG);
    if (rv == -1) {
      if (errno == EINTR)  continue;
      if (errno == ECHILD) {
	__sensor_server_pid = -1;
	break;
      }
    } else if (rv > 0) {
      __sensor_server_pid = -1;
      break;
    }
  }

  if (__sensor_server_pid != -1) {
    logger->log_warn(name(), "Killing XnSensorServer");
    ::kill(__sensor_server_pid, SIGKILL);
    __sensor_server_pid = -1;
  }
}
