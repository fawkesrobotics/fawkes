
/***************************************************************************
 *  arm_kindrv.h - Class for a Kinova Jaco arm, using libkindrv
 *
 *  Created: Tue Jul 29 14:58:32 2014
 *  Copyright  2014  Bahram Maleki-Fard
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

#ifndef __PLUGINS_JACO_ARM_KINDRV_H_
#define __PLUGINS_JACO_ARM_KINDRV_H_

#include "arm.h"
#include "types.h"

namespace KinDrv {
  class JacoArm;
}

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

//class RefPtr;

class JacoArmKindrv : public JacoArm
{
 public:
  JacoArmKindrv(const char *name = NULL);
  virtual ~JacoArmKindrv();

  virtual void initialize();

  virtual bool final();
  virtual bool initialized();

  virtual void get_joints(std::vector<float> &to) const;
  virtual void get_coords(std::vector<float> &to);
  virtual void get_fingers(std::vector<float> &to) const;

  virtual void stop();
  virtual void push_joystick(unsigned int button);
  virtual void release_joystick();

  virtual void goto_trajec(std::vector< std::vector<float> >* trajec, std::vector<float> &fingers);
  virtual void goto_joints(std::vector<float> &joints, std::vector<float> &fingers, bool followup=false);
  virtual void goto_coords(std::vector<float> &coords, std::vector<float> &fingers);
  virtual void goto_ready();
  virtual void goto_retract();

 private:
  KinDrv::JacoArm  *__arm;

  fawkes::jaco_target_type_t __target_type;
  bool __final;

  bool __ctrl_ang;
};



} // end of namespace fawkes

#endif
