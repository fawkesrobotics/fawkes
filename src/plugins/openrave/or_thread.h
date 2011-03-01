
/***************************************************************************
 *  or_thread.h - OpenRAVE Thread
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#ifndef __PLUGINS_OPENRAVE_OR_THREAD_H_
#define __PLUGINS_OPENRAVE_OR_THREAD_H_

#include <plugins/openrave/aspect/or_connector.h>
#include <plugins/openrave/aspect/or_inifin.h>

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/aspect_provider.h>

namespace fawkes {
  class OpenRAVEEnvironment;
  class OpenRAVERobot;
  class OpenRAVEManipulator;
}

class OpenRAVEThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::AspectProviderAspect,
  public fawkes::OpenRAVEConnector
{
 public:
  OpenRAVEThread();
  virtual ~OpenRAVEThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  //for OpenRAVEConnector
  //virtual void testDebug();
  virtual fawkes::OpenRAVEEnvironment*	getEnvironment() const;
  virtual fawkes::OpenRAVERobot*	getActiveRobot() const;
  virtual void				setActiveRobot(fawkes::OpenRAVERobot* robot);
  virtual fawkes::OpenRAVERobot*	addRobot(const std::string& filenameRobot, bool autogenerateIK);
  virtual void 				setManipulator(fawkes::OpenRAVEManipulator* manip, float transX=0.f, float transY=0.f, float transZ=0.f, bool calibrate=0);
  virtual void 				setManipulator(fawkes::OpenRAVERobot* robot, fawkes::OpenRAVEManipulator* manip, float transX=0.f, float transY=0.f, float transZ=0.f, bool calibrate=0);

  virtual void startViewer() const;
  virtual void runPlanner(fawkes::OpenRAVERobot* = NULL);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::OpenRAVEAspectIniFin     __or_aspectIniFin;

  fawkes::OpenRAVEEnvironment*  __OREnv;
  fawkes::OpenRAVERobot*        __ORRobot;
};

#endif
