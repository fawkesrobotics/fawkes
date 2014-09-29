
/***************************************************************************
 *  bimanual_openrave_thread.cpp - Jaco plugin OpenRAVE Thread for bimanual manipulation
 *
 *  Created: Mon Jul 28 19:43:20 2014
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

#include "bimanual_openrave_thread.h"
#include "types.h"
#include "arm.h"

#include <interfaces/JacoInterface.h>
#include <core/threading/mutex.h>

#include <cmath>
#include <stdio.h>
#include <cstring>
#include <algorithm>

#ifdef HAVE_OPENRAVE
 #include <openrave/openrave.h>
 #include <plugins/openrave/environment.h>
 #include <plugins/openrave/robot.h>
 #include <plugins/openrave/manipulator.h>
 #include <plugins/openrave/manipulators/kinova_jaco.h>
 using namespace OpenRAVE;
#endif

using namespace fawkes;
using namespace std;

/** @class JacoBimanualOpenraveThread "bimanual_openrave_thread.h"
 * Jaco Arm thread for dual-arm setup, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
JacoBimanualOpenraveThread::JacoBimanualOpenraveThread(jaco_arm_t *arm_l, jaco_arm_t *arm_r)
  : JacoOpenraveBaseThread("JacoBimanualOpenraveThread")
{
  __arms.left.arm = arm_l;
  __arms.right.arm = arm_r;
#ifdef HAVE_OPENRAVE
  __planner_env.env   = NULL;
  __planner_env.robot = NULL;
  __planner_env.manip = NULL;
#endif
}

void
JacoBimanualOpenraveThread::_init()
{
#ifdef HAVE_OPENRAVE
  __arms.left.manipname  = config->get_string("/hardware/jaco/openrave/manipname/dual_left");
  __arms.right.manipname = config->get_string("/hardware/jaco/openrave/manipname/dual_right");
#endif
}

void
JacoBimanualOpenraveThread::_load_robot()
{
#ifdef HAVE_OPENRAVE
  __cfg_OR_robot_file  = config->get_string("/hardware/jaco/openrave/robot_dual_file");

  try {
    //__viewer_env.robot = openrave->add_robot(__cfg_OR_robot_file, false);
    // manually add robot; the automatic needs to be altered
    __viewer_env.robot = new OpenRaveRobot(logger);
    __viewer_env.robot->load(__cfg_OR_robot_file, __viewer_env.env);
    __viewer_env.env->add_robot(__viewer_env.robot);
    __viewer_env.robot->set_ready();
    openrave->set_active_robot(__viewer_env.robot);
  } catch (Exception& e) {
    throw fawkes::Exception("Could not add robot '%s' to openrave environment. (Error: %s)", __cfg_OR_robot_file.c_str(), e.what_no_backtrace());
  }

  try {
    __viewer_env.manip = new OpenRaveManipulatorKinovaJaco(6, 6);
    __viewer_env.manip->add_motor(0,0);
    __viewer_env.manip->add_motor(1,1);
    __viewer_env.manip->add_motor(2,2);
    __viewer_env.manip->add_motor(3,3);
    __viewer_env.manip->add_motor(4,4);
    __viewer_env.manip->add_motor(5,5);

    // Set manipulator and offsets.
    openrave->set_manipulator(__viewer_env.robot, __viewer_env.manip, 0.f, 0.f, 0.f);

    EnvironmentMutex::scoped_lock lock(__viewer_env.env->get_env_ptr()->GetMutex());

    __arms.right.manip = __viewer_env.robot->get_robot_ptr()->SetActiveManipulator(__arms.right.manipname);
    if( __cfg_OR_auto_load_ik ) {
      logger->log_debug(name(), "load IK for right arm");
      __viewer_env.env->load_IK_solver(__viewer_env.robot, OpenRAVE::IKP_Transform6D);
    }

    __arms.left.manip = __viewer_env.robot->get_robot_ptr()->SetActiveManipulator(__arms.left.manipname);
    if( __cfg_OR_auto_load_ik ) {
      logger->log_debug(name(), "load IK for left arm");
      __viewer_env.env->load_IK_solver(__viewer_env.robot, OpenRAVE::IKP_Transform6D);
    }

  } catch (Exception& e) {
    finalize();
    throw;
  }
#endif
}


void
JacoBimanualOpenraveThread::once()
{
#ifdef HAVE_OPENRAVE
  // create cloned environment for planning
  logger->log_debug(name(), "Clone environment for planning");
  openrave->clone(__planner_env.env, __planner_env.robot, __planner_env.manip);

  if( !__planner_env.env || !__planner_env.robot || !__planner_env.manip) {
    throw fawkes::Exception("Could not clone properly, received a NULL pointer");
  }

  // load dualmanipulation module
  EnvironmentMutex::scoped_lock lock(__planner_env.env->get_env_ptr()->GetMutex());
  __mod_dualmanip = RaveCreateModule(__planner_env.env->get_env_ptr(), "dualmanipulation");
  __planner_env.env->get_env_ptr()->Add( __mod_dualmanip, true, "");

  // load MultiManipIkSolver stuff
  // Get all the links that are affecte by left/right manipulator
  vector<int> arm_idx_l = __arms.left.manip->GetArmIndices();
  vector<int> arm_idx_r = __arms.right.manip->GetArmIndices();
  vector<int> grp_idx = __arms.left.manip->GetGripperIndices();
  arm_idx_l.reserve( arm_idx_l.size() + grp_idx.size() );
  arm_idx_l.insert( arm_idx_l.end(), grp_idx.begin(), grp_idx.end() );
  grp_idx = __arms.right.manip->GetGripperIndices();
  arm_idx_r.reserve( arm_idx_r.size() + grp_idx.size() );
  arm_idx_r.insert( arm_idx_r.end(), grp_idx.begin(), grp_idx.end() );

  RobotBasePtr robot = __planner_env.robot->get_robot_ptr();
  vector<KinBody::LinkPtr> all_links = robot->GetLinks();
  for( vector<KinBody::LinkPtr>::iterator link=all_links.begin(); link!=all_links.end(); ++link ) {
    bool affected = false;
    for( vector<int>::iterator idx=arm_idx_l.begin(); idx!=arm_idx_l.end(); ++idx ) {
      if( robot->DoesAffect(robot->GetJointFromDOFIndex(*idx)->GetJointIndex(),(*link)->GetIndex()) ) {
        // this link is affected by left manipulator
        links_left_.insert(*link);
        arm_idx_l.erase(idx); // no need to check this one again
        affected = true;
        break;
      }
    }

    if( affected )
      continue;

    for( vector<int>::iterator idx=arm_idx_r.begin(); idx!=arm_idx_r.end(); ++idx ) {
      if( robot->DoesAffect(robot->GetJointFromDOFIndex(*idx)->GetJointIndex(),(*link)->GetIndex()) ) {
        // this link is affected by right manipulator
        links_right_.insert(*link);
        arm_idx_r.erase(idx); // no need to check this one again
        break;
      }
    }
  }
#endif
}

void
JacoBimanualOpenraveThread::finalize() {
  __arms.left.arm = NULL;
  __arms.right.arm = NULL;
#ifdef HAVE_OPENRAVE
  openrave->set_active_robot( NULL );

  __planner_env.env->get_env_ptr()->Remove( __mod_dualmanip );
  __planner_env.robot = NULL;
  __planner_env.manip = NULL;
  __planner_env.env = NULL;
#endif

  JacoOpenraveBaseThread::finalize();
}

bool
JacoBimanualOpenraveThread::add_target(float l_x, float l_y, float l_z, float l_e1, float l_e2, float l_e3,
                                       float r_x, float r_y, float r_z, float r_e1, float r_e2, float r_e3)
{
  // no IK-solving for coordinated bimanual movement implemented yet
  return false;
}

void
JacoBimanualOpenraveThread::update_openrave()
{
  // do nothing, this thread is only for plannning!
}

/** Plot the first target of the queue in the viewer_env */
void
JacoBimanualOpenraveThread::plot_first()
{
}

bool
JacoBimanualOpenraveThread::_plan_path()
{
  return true;
}

bool
JacoBimanualOpenraveThread::_solve_multi_ik(vector<dReal> &left, vector<dReal> &right)
{
  EnvironmentMutex::scoped_lock plan_lock(__planner_env.env->get_env_ptr()->GetMutex());

  // robot ptr for convenienc
  RobotBasePtr robot = __planner_env.robot->get_robot_ptr();

  // get grabbed bodies
  vector<KinBodyPtr> grabbed;
  robot->GetGrabbed(grabbed);

  // save state of grabbed bodies
  vector<KinBody::KinBodyStateSaver> statesaver;
  for( vector<KinBodyPtr>::iterator body=grabbed.begin(); body!=grabbed.end(); ++body ) {
    statesaver.push_back(KinBody::KinBodyStateSaver(*body));
  }

  // get IK solutions for both arms
  vector< vector<dReal> > solutions_l, solutions_r;
  {
    // save state of robot
    RobotBase::RobotStateSaver robot_saver(robot);

    vector<KinBody::LinkPtr> all_links = robot->GetLinks();
    // Find IK solutions for left arm
    // Disable all links of right manipulator
    for( set<KinBody::LinkPtr>::iterator body=links_right_.begin(); body!=links_right_.end(); ++body) {
      (*body)->Enable(false);
    }
    // Enable only grabbed bodies of this manipulator
    for( vector<KinBodyPtr>::iterator body=grabbed.begin(); body!=grabbed.end(); ++body) {
      (*body)->Enable(__arms.left.manip->IsGrabbing(*body));
    }
    // Get Ik Solutions.
    __planner_env.robot->set_target_euler(EULER_ZXZ,
                                          __arms.left.target->pos.at(0), __arms.left.target->pos.at(1), __arms.left.target->pos.at(2),
                                          __arms.left.target->pos.at(3), __arms.left.target->pos.at(4), __arms.left.target->pos.at(5));
    IkParameterization param = __planner_env.robot->get_target().ikparam;
    __arms.left.manip->FindIKSolutions(param, solutions_l, IKFO_CheckEnvCollisions);
    if( solutions_l.empty() ) {
      logger->log_warn(name(), "No IK solutions found for left arm");
      return false;
    }

    // now same for right arm. but enable links of right manipulator first
    for( set<KinBody::LinkPtr>::iterator body=links_right_.begin(); body!=links_right_.end(); ++body) {
      (*body)->Enable(true);
    }
    // Disable all links of left manipulator
    for( set<KinBody::LinkPtr>::iterator body=links_left_.begin(); body!=links_left_.end(); ++body) {
      (*body)->Enable(false);
    }
    // Enable only grabbed bodies of this manipulator
    for( vector<KinBodyPtr>::iterator body=grabbed.begin(); body!=grabbed.end(); ++body) {
      (*body)->Enable(__arms.right.manip->IsGrabbing(*body));
    }
    // Get Ik Solutions.
    __planner_env.robot->set_target_euler(EULER_ZXZ,
                                          __arms.right.target->pos.at(0), __arms.right.target->pos.at(1), __arms.right.target->pos.at(2),
                                          __arms.right.target->pos.at(3), __arms.right.target->pos.at(4), __arms.right.target->pos.at(5));
    param = __planner_env.robot->get_target().ikparam;
    __arms.right.manip->FindIKSolutions(param, solutions_r, IKFO_CheckEnvCollisions);
    if( solutions_r.empty() ) {
      logger->log_warn(name(), "No IK solutions found for right arm");
      return false;
    }
  } // robot state-saver destroyed

  // restore kinbody states
  for( vector<KinBody::KinBodyStateSaver>::iterator s=statesaver.begin(); s!=statesaver.end(); ++s ) {
    (*s).Restore();
  }

  // finally find the solutions without collision and store them
  // TODO: sort by distance first?
  {
    // save state of robot
    RobotBase::RobotStateSaver robot_saver(robot);
    vector< vector<dReal> >::iterator sol_l, sol_r;

    // try each combination to find first non-colliding
    for( sol_l=solutions_l.begin(); sol_l!=solutions_l.end(); ++sol_l ) {
      for( sol_r=solutions_r.begin(); sol_r!=solutions_r.end(); ++sol_r ) {
        // set joints for robot model
        robot->SetDOFValues(*sol_l, 1, __arms.left.manip->GetGripperIndices());
        robot->SetDOFValues(*sol_r, 1, __arms.right.manip->GetGripperIndices());

        // check for collisions
        if( !robot->CheckSelfCollision() && !robot->GetEnv()->CheckCollision(robot) ) {
          left.clear();
          right.clear();
          left = *sol_l;
          right = *sol_r;
          return true;
        }
      }
    }
  } // robot state-saver destroyed


  return false;
}
