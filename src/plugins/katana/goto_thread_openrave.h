
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

#include <plugins/openrave/aspect/or.h>

#ifdef HAVE_OPENRAVE
namespace fawkes {
  class OpenRAVERobot;
  class OpenRAVEManipulatorKatana6M180;
}
#endif

class KatanaGotoThreadOpenRAVE : public KatanaGotoThread
{
 public:
  KatanaGotoThreadOpenRAVE(fawkes::RefPtr<CLMBase> katana, fawkes::Logger *logger, fawkes::OpenRAVEConnector* openrave,
		   unsigned int poll_interval_ms,
                   std::string robot_file,
                   bool autoload_IK,
                   bool use_viewer);

#ifdef HAVE_OPENRAVE

  virtual void once();
  virtual void init();
  virtual void finalize();

  void set_target(float x, float y, float z, float phi, float theta, float psi);
  void set_target(const std::string& object_name);

  virtual bool update_motor_data();
  virtual void move_katana();

 private:
  fawkes::OpenRAVERobot*                        __OR_robot;
  fawkes::OpenRAVEManipulatorKatana6M180*       __OR_manip;

  std::string                                   __target_object;
  std::vector< std::vector<float> >*            __target_traj;
  std::vector< std::vector<float> >::iterator   __it;

  std::vector< int >    __motor_encoders;
  std::vector< float >  __motor_angles;

  const std::string     __cfg_robot_file;
  bool                  __cfg_autoload_IK;
  bool                  __cfg_use_viewer;

  bool                  __is_target_object;

  fawkes::OpenRAVEConnector*    _openrave;

#endif //HAVE_OPENRAVE
};

#endif
