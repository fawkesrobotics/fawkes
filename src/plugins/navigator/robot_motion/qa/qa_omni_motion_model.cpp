
/***************************************************************************
 *  qa_omni_motion_model.cpp - OmniMotionModel QA
 *
 *  Generated: Thu April 03 14:29:33 2008
 *  Copyright  2008  Daniel Beck
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

/// @cond QA

#include <plugins/navigator/robot_motion/omni_motion_model.h>

#include <cstdio>
#include <cmath>

int main(int argc, char** argv)
{
  OmniMotionModel omm(300.0, 50.0, 0.2);
  
  float vx    = 0.0;//M_PI / 2.0;
  float vy    = M_PI / 2.0;
  float omega = M_PI / 2.0;

  float rpm_right;
  float rpm_rear;
  float rpm_left;
  float dx;
  float dy;
  float dphi;

  omm.velocities_to_rpm(vx, vy, omega, rpm_right, rpm_rear, rpm_left);
  printf("vx=%.2f vy=%.2f omega=%.2f rpm_right=%.2f rpm_rear=%.2f rpm_left=%.2f\n",
	 vx, vy, omega, rpm_right, rpm_rear, rpm_left);
  
  omm.update_odometry(rpm_right/60.0, rpm_rear/60.0, rpm_left/60.0, 1.0);
  omm.get_odom_pose(dx, dy, dphi);
  omm.get_actual_velocities(vx, vy, omega);
  printf("act_vx=%.2f act_vy=%.2f act_omega=%.2f dx=%.2f dy=%.2f dphi=%.2f\n\n", 
	 vx, vy, omega, dx, dy, dphi);
  
  omm.reset_odometry();

  omm.pose_to_rpm(dx, dy, dphi, 1.0, rpm_right, rpm_rear, rpm_left);
  printf("dx=%.2f dy=%.2f dphi=%.2f rpm_right=%.2f rpm_rear=%.2f rpm_left=%.2f\n",
	 dx, dy, dphi, rpm_right, rpm_rear, rpm_left);

  omm.update_odometry(rpm_right/60.0, rpm_rear/60.0, rpm_left/60.0, 1.0);
  omm.get_odom_pose(dx, dy, dphi);
  omm.get_actual_velocities(vx, vy, omega);
  printf("act_vx=%.2f act_vy=%.2f act_omega=%.2f dx=%.2f dy=%.2f dphi=%.2f\n", 
	 vx, vy, omega, dx, dy, dphi);

  return 0;
}

/// @endcond
