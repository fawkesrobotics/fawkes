
/***************************************************************************
 *  arm_dummy.h - Class for a Kinova Jaco arm, simulating a dummy arm
 *
 *  Created: Mon Aug 04 19:58:22 2014
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

#ifndef __PLUGINS_JACO_ARM_DUMMY_H_
#define __PLUGINS_JACO_ARM_DUMMY_H_

#include "arm.h"
#include "types.h"


namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

//class RefPtr;

class JacoArmDummy : public JacoArm
{
 public:
  JacoArmDummy(const char *name);
  virtual ~JacoArmDummy();

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

  std::vector<float> __coords;
  std::vector<float> __joints;
  std::vector<float> __fingers;

  std::vector<float> __pos_ready;
  std::vector<float> __pos_retract;

};


} // end of namespace fawkes

#endif
