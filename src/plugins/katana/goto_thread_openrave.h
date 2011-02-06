
/***************************************************************************
 *  goto_thread_openrave.h - Katana goto one-time thread using openrave lib
 *
 *  Created: Wed Jun 10 11:44:24 2009
 *  Copyright  2006-2009  Tim Niemueller [www.niemueller.de]
 *                  2010  Bahram Maleki-Fard
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

#ifndef __PLUGINS_KATANA_GOTO_THREAD_OPENRAVE_H_
#define __PLUGINS_KATANA_GOTO_THREAD_OPENRAVE_H_

#include "goto_thread.h"

#include <vector>
#include <string>

#ifdef HAVE_OPENRAVE
namespace fawkes {
  class OpenRAVEConnector;
  class OpenRAVEManipulatorKatana6M180;
}
#endif

class KatanaGotoThreadOpenRAVE : public KatanaGotoThread
{
 public:
  KatanaGotoThreadOpenRAVE(fawkes::RefPtr<CLMBase> katana, fawkes::Logger *logger,
		   unsigned int poll_interval_ms,
                   std::string robotFile,
                   bool autoloadIK,
                   bool useViewer);

#ifdef HAVE_OPENRAVE

  virtual void once();
  virtual void init();
  virtual void finalize();

  void set_target(float x, float y, float z, float phi, float theta, float psi);

  virtual bool updateMotorData();
  virtual void moveKatana();

 private:
  fawkes::OpenRAVEConnector*                    __ORCon;
  fawkes::OpenRAVEManipulatorKatana6M180*       __manip;

  std::vector< std::vector<float> >*            __targetTraj;
  std::vector< std::vector<float> >::iterator   __it;

  std::vector< int >    __motorEncoders;
  std::vector< float >  __motorAngles;

  const std::string     __cfg_robotFile;
  bool                  __cfg_autoloadIK;
  bool                  __cfg_useViewer;

#endif //HAVE_OPENRAVE
};

#endif
