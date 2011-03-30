
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
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR),
    AspectProviderAspect("OpenNiAspect", &__openni_aspect_inifin)
{
  set_prepfin_conc_loop(true);
}


/** Destructor. */
OpenNiContextThread::~OpenNiContextThread()
{
}


void
OpenNiContextThread::init()
{
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
}


void
OpenNiContextThread::finalize()
{
  __openni->StopGeneratingAll();
  __openni->Shutdown();
  __openni.clear();
  __openni_aspect_inifin.set_openni_context(__openni);
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
	if (! generator.IsGenerating() && (pnd.Type != XN_NODE_TYPE_DEVICE) ) {
	  logger->log_warn(name(), "Inactive node '%s' (%s, %s/%s), trying to activate",
			   (*n).GetInstanceName(), type_to_string(pnd.Type),
			   pnd.strVendor, pnd.strName);
	  generator.StartGenerating();
	} else if (pnd.Type == XN_NODE_TYPE_DEVICE) {
	  // as an alternative, verify how often it has not been updated
	  if (generator.IsDataNew()) {
	    __device_no_data_loops = 0;
	  } else {
	    if (++__device_no_data_loops > 3) {
	      logger->log_warn(name(), "Device '%s' had no fresh data for long time. "
			       "Reload maybe necessary.", (*n).GetInstanceName());
	    }
	  }
	}

	xn::ErrorStateCapability ecap = generator.GetErrorStateCap();
	if (ecap.GetErrorState() != XN_STATUS_OK) {
	  logger->log_warn(name(), "ERROR in node '%s': %s", (*n).GetInstanceName(),
			   xnGetStatusString(ecap.GetErrorState()));
	}
      }
    }
  }
}
