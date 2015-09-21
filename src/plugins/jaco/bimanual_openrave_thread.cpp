
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

#include <core/threading/mutex.h>

#include <cmath>
#include <stdio.h>
#include <cstring>
#include <algorithm>
#include <unistd.h>

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
 * @param arms pointer to jaco_dual_arm_t struct, to be used in this thread
 */
JacoBimanualOpenraveThread::JacoBimanualOpenraveThread(jaco_dual_arm_t *arms)
  : JacoOpenraveBaseThread("JacoBimanualOpenraveThread")
{
  __arms.left.arm = arms->left;
  __arms.right.arm = arms->right;
#ifdef HAVE_OPENRAVE
  __planner_env.env   = NULL;
  __planner_env.robot = NULL;
  __planner_env.manip = NULL;
#endif

  __constrained = false;
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

  // create cloned environment for planning
  logger->log_debug(name(), "Clone environment for planning");
  openrave->clone(__planner_env.env, __planner_env.robot, __planner_env.manip);

  if( !__planner_env.env || !__planner_env.robot || !__planner_env.manip) {
    throw fawkes::Exception("Could not clone properly, received a NULL pointer");
  }

  // set name of env
  __planner_env.env->set_name("Planner_Bimanual");

  // set manips to those of planner env
  __arms.right.manip = __planner_env.robot->get_robot_ptr()->SetActiveManipulator(__arms.right.manipname);
  __arms.left.manip = __planner_env.robot->get_robot_ptr()->SetActiveManipulator(__arms.left.manipname);

  // initial modules for dualmanipulation
  _init_dualmanipulation();
#endif
}


void
JacoBimanualOpenraveThread::_init_dualmanipulation()
{
#ifdef HAVE_OPENRAVE
  // load dualmanipulation module
  EnvironmentMutex::scoped_lock lock(__planner_env.env->get_env_ptr()->GetMutex());
  __mod_dualmanip = RaveCreateModule(__planner_env.env->get_env_ptr(), "dualmanipulation");
  __planner_env.env->get_env_ptr()->Add( __mod_dualmanip, true, __planner_env.robot->get_robot_ptr()->GetName());

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


void
JacoBimanualOpenraveThread::loop()
{
#ifndef HAVE_OPENRAVE
  usleep(30e3);
#else
  if( __arms.left.arm == NULL || __arms.right.arm == NULL ) {
    usleep(30e3);
    return;
  }

  // get first target in queues
  __arms.left.arm->target_mutex->lock();
  __arms.right.arm->target_mutex->lock();
  if( !__arms.left.arm->target_queue->empty() && !__arms.right.arm->target_queue->empty() ) {
    __arms.left.target  = __arms.left.arm->target_queue->front();
    __arms.right.target = __arms.right.arm->target_queue->front();
  }
  __arms.left.arm->target_mutex->unlock();
  __arms.right.arm->target_mutex->unlock();

  if( !__arms.left.target || !__arms.right.target
   || !__arms.left.target->coord || !__arms.right.target->coord
   || __arms.left.target->trajec_state !=TRAJEC_WAITING
   || __arms.right.target->trajec_state!=TRAJEC_WAITING ) {
    //no new target in queue, or target is not meant for coordinated bimanual manipulation
    usleep(30e3);
    return;
  }

  // copy environment first
  _copy_env();

  // get suiting IK solutions
  vector<float> sol_l, sol_r;
  bool solvable = _solve_multi_ik(sol_l, sol_r);
  __arms.left.arm->target_mutex->lock();
  __arms.right.arm->target_mutex->lock();
  if( !solvable ) {
    __arms.left.target->trajec_state=TRAJEC_IK_ERROR;
    __arms.right.target->trajec_state=TRAJEC_IK_ERROR;
    __arms.left.arm->target_mutex->unlock();
    __arms.right.arm->target_mutex->unlock();
    usleep(30e3);
    return;
  } else {
    __arms.left.target->type=TARGET_ANGULAR;
    __arms.left.target->pos = sol_l;
    __arms.right.target->type=TARGET_ANGULAR;
    __arms.right.target->pos = sol_r;
    __arms.left.arm->target_mutex->unlock();
    __arms.right.arm->target_mutex->unlock();

    // run path planner
    _plan_path();
  }

#endif
}

/** Enable/Disable constrained planning.
 * Enabling it will constrain the movement, which means it is tried to
 * maintain the distance of the grippers to each other. This should be
 * activated when moving an object with both hands, but disabled in
 * situations when the arms do not need to hold the object simultaneously
 * at all times.
 *
 * @param enable Enables/Disables the state.
 */
void
JacoBimanualOpenraveThread::set_constrained(bool enable)
{
  __constrained = enable;
}

/** Add target for coordinated manipulation to the queue.
 *
 * This adds targets to the queues for both left and right arms. It sets
 * the target->coord flag to "true", which means it will not be processed
 * by the threads for uncoordinated manipulation!
 *
 * @param l_x x-coordinate of target position of left arm
 * @param l_y y-coordinate of target position of left arm
 * @param l_z z-coordinate of target position of left arm
 * @param l_e1 1st euler rotation of target orientation of left arm
 * @param l_e2 2nd euler rotation of target orientation of left arm
 * @param l_e3 3rd euler rotation of target orientation of left arm
 * @param r_x x-coordinate of target position of right arm
 * @param r_y y-coordinate of target position of right arm
 * @param r_z z-coordinate of target position of right arm
 * @param r_e1 1st euler rotation of target orientation of right arm
 * @param r_e2 2nd euler rotation of target orientation of right arm
 * @param r_e3 3rd euler rotation of target orientation of right arm
 * @return "true", if target could be added to queue.
 */
bool
JacoBimanualOpenraveThread::add_target(float l_x, float l_y, float l_z, float l_e1, float l_e2, float l_e3,
                                       float r_x, float r_y, float r_z, float r_e1, float r_e2, float r_e3)
{
#ifdef HAVE_OPENRAVE
  // no IK checking yet, just enqueue until they can be processed
  // create new targets for the queues
  RefPtr<jaco_target_t> target_l(new jaco_target_t());
  target_l->type = TARGET_CARTESIAN;
  target_l->trajec_state = TRAJEC_WAITING;
  target_l->coord=true;
  target_l->pos.push_back(l_x);
  target_l->pos.push_back(l_y);
  target_l->pos.push_back(l_z);
  target_l->pos.push_back(l_e1);
  target_l->pos.push_back(l_e2);
  target_l->pos.push_back(l_e3);

  RefPtr<jaco_target_t> target_r(new jaco_target_t());
  target_r->type = TARGET_CARTESIAN;
  target_r->trajec_state = TRAJEC_WAITING;
  target_r->coord=true;
  target_r->pos.push_back(r_x);
  target_r->pos.push_back(r_y);
  target_r->pos.push_back(r_z);
  target_r->pos.push_back(r_e1);
  target_r->pos.push_back(r_e2);
  target_r->pos.push_back(r_e3);

  __arms.left.arm->target_mutex->lock();
  __arms.right.arm->target_mutex->lock();
  __arms.left.arm->target_queue->push_back(target_l);
  __arms.right.arm->target_queue->push_back(target_r);
  __arms.left.arm->target_mutex->unlock();
  __arms.right.arm->target_mutex->unlock();

  return true;
#else
  return false;
#endif
}

void
JacoBimanualOpenraveThread::update_openrave()
{
  // do nothing, this thread is only for plannning!
}

void
JacoBimanualOpenraveThread::plot_first()
{
}

void
JacoBimanualOpenraveThread::_set_trajec_state(jaco_trajec_state_t state)
{
#ifdef HAVE_OPENRAVE
  __arms.left.arm->target_mutex->lock();
  __arms.right.arm->target_mutex->lock();
  __arms.left.target->trajec_state=state;
  __arms.right.target->trajec_state=state;
  __arms.left.arm->target_mutex->unlock();
  __arms.right.arm->target_mutex->unlock();
#endif
}

void
JacoBimanualOpenraveThread::_copy_env()
{
#ifdef HAVE_OPENRAVE
  // Update bodies in planner-environment
  // clone robot state, ignoring grabbed bodies
  {
    EnvironmentMutex::scoped_lock view_lock(__viewer_env.env->get_env_ptr()->GetMutex());
    EnvironmentMutex::scoped_lock plan_lock(__planner_env.env->get_env_ptr()->GetMutex());
    __planner_env.robot->get_robot_ptr()->ReleaseAllGrabbed();
    __planner_env.env->delete_all_objects();

    /*
    // Old method. Somehow we encountered problems. OpenRAVE internal bug?
    RobotBase::RobotStateSaver saver(__viewer_env.robot->get_robot_ptr(),
                                     0xffffffff&~KinBody::Save_GrabbedBodies&~KinBody::Save_ActiveManipulator&~KinBody::Save_ActiveDOF);
    saver.Restore( __planner_env.robot->get_robot_ptr() );
    */
    // New method. Simply set the DOF values as they are in __viewer_env
    vector<dReal> dofs;
    __viewer_env.robot->get_robot_ptr()->GetDOFValues(dofs);
    __planner_env.robot->get_robot_ptr()->SetDOFValues(dofs);
  }

  // then clone all objects
  __planner_env.env->clone_objects( __viewer_env.env );

  // update robot state with attached objects
  {
    EnvironmentMutex::scoped_lock lock(__planner_env.env->get_env_ptr()->GetMutex());
    EnvironmentMutex::scoped_lock view_lock(__viewer_env.env->get_env_ptr()->GetMutex());
    /*
    // Old method. Somehow we encountered problems. OpenRAVE internal bug?
    RobotBase::RobotStateSaver saver(__viewer_env.robot->get_robot_ptr(),
                                     KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_GrabbedBodies);
    saver.Restore( __planner_env.robot->get_robot_ptr() );
    */
    // New method. Grab all bodies in __planner_env that are grabbed in __viewer_env by this manipulator
    vector<RobotBase::GrabbedInfoPtr> grabbed;
    __viewer_env.robot->get_robot_ptr()->GetGrabbedInfo(grabbed);
    for( vector<RobotBase::GrabbedInfoPtr>::iterator it=grabbed.begin(); it!=grabbed.end(); ++it ) {
      logger->log_debug(name(), "compare _robotlinkname '%s' with our manip links '%s' and '%s'",
                        (*it)->_robotlinkname.c_str(),
                        __arms.left.manip->GetEndEffector()->GetName().c_str(),
                        __arms.right.manip->GetEndEffector()->GetName().c_str());
      if( (*it)->_robotlinkname == __arms.left.manip->GetEndEffector()->GetName() ) {
        logger->log_debug(name(), "attach '%s' to '%s'!", (*it)->_grabbedname.c_str(), __arms.left.manip->GetEndEffector()->GetName().c_str());
        __planner_env.robot->attach_object((*it)->_grabbedname.c_str(), __planner_env.env, __arms.left.manipname.c_str());

      } else if( (*it)->_robotlinkname == __arms.right.manip->GetEndEffector()->GetName() ) {
        logger->log_debug(name(), "attach '%s' to '%s'!", (*it)->_grabbedname.c_str(), __arms.right.manip->GetEndEffector()->GetName().c_str());
        __planner_env.robot->attach_object((*it)->_grabbedname.c_str(), __planner_env.env, __arms.right.manipname.c_str());
      }
    }
  }
#endif
}

bool
JacoBimanualOpenraveThread::_plan_path()
{
#ifdef HAVE_OPENRAVE
  _set_trajec_state(TRAJEC_PLANNING);

  EnvironmentMutex::scoped_lock lock(__planner_env.env->get_env_ptr()->GetMutex());

  // Set  active DOFs
  vector<int> dofs   = __arms.left.manip->GetArmIndices();
  vector<int> dofs_r = __arms.right.manip->GetArmIndices();
  dofs.reserve(dofs.size() + dofs_r.size());
  dofs.insert(dofs.end(), dofs_r.begin(), dofs_r.end());
  __planner_env.robot->get_robot_ptr()->SetActiveDOFs(dofs);

  // setup command for dualmanipulation module
  stringstream cmdin,cmdout;
  cmdin << std::setprecision(numeric_limits<dReal>::digits10+1);
  cmdout << std::setprecision(numeric_limits<dReal>::digits10+1);

  vector<dReal> sol;
  cmdin << "MoveAllJoints goal";
  __planner_env.manip->set_angles_device(__arms.left.target->pos);
  __planner_env.manip->get_angles(sol);
  for(size_t i = 0; i < sol.size(); ++i) {
    cmdin << " " << sol[i];
  }
  __planner_env.manip->set_angles_device(__arms.right.target->pos);
  __planner_env.manip->get_angles(sol);
  for(size_t i = 0; i < sol.size(); ++i) {
    cmdin << " " << sol[i];
  }

  //add additional planner parameters
  if( !__plannerparams.empty() ) {
    cmdin << " " << __plannerparams;
  }

  //constrain planning if required
  if( __constrained ) {
    cmdin << " constrainterrorthresh 1";
  }

  cmdin << " execute 0";
  cmdin << " outputtraj";
  //logger->log_debug(name(), "Planner: dualmanip cmdin:%s", cmdin.str().c_str());

  // plan path
  bool success = false;
  try {
    success = __mod_dualmanip->SendCommand(cmdout,cmdin);
  } catch(openrave_exception &e) {
    logger->log_debug(name(), "Planner: dualmanip command failed. Ex:%s", e.what());
  }

  if(!success) {
    logger->log_warn(name(),"Planner: planning failed");
    _set_trajec_state(TRAJEC_PLANNING_ERROR);
    __arms.left.arm->target_mutex->lock();
    __arms.right.arm->target_mutex->lock();
    __arms.left.arm->target_mutex->unlock();
    __arms.right.arm->target_mutex->unlock();
    return false;

  } else {
    //logger->log_debug(name(), "Planner: path planned. cmdout:%s", cmdout.str().c_str());

    // read returned trajectory
    ConfigurationSpecification cfg_spec = __planner_env.robot->get_robot_ptr()->GetActiveConfigurationSpecification();
    TrajectoryBasePtr traj = RaveCreateTrajectory(__planner_env.env->get_env_ptr(), "");
    traj->Init(cfg_spec);
    if( !traj->deserialize(cmdout) ) {
      logger->log_warn(name(), "Planner: Cannot read trajectory data.");
      _set_trajec_state(TRAJEC_PLANNING_ERROR);
      __arms.left.arm->target_mutex->lock();
      __arms.right.arm->target_mutex->lock();
      __arms.left.arm->target_mutex->unlock();
      __arms.right.arm->target_mutex->unlock();
      return false;

    } else {
      // sampling trajectory and setting target trajectory
      jaco_trajec_t* trajec_l = new jaco_trajec_t();
      jaco_trajec_t* trajec_r = new jaco_trajec_t();
      jaco_trajec_point_t p; // point we will add to trajectories
      vector<dReal> tmp_p;
      int arm_dof = cfg_spec.GetDOF() / 2;

      for(dReal time = 0; time <= traj->GetDuration(); time += (dReal)__cfg_OR_sampling) {
        vector<dReal> point;
        traj->Sample(point,time);

        tmp_p = vector<dReal>(point.begin(), point.begin()+arm_dof);
        __planner_env.manip->angles_or_to_device( tmp_p, p);
        trajec_l->push_back(p);

        tmp_p = vector<dReal>(point.begin()+arm_dof, point.begin()+2*arm_dof);
        __planner_env.manip->angles_or_to_device( tmp_p, p);
        trajec_r->push_back(p);
      }

      __arms.left.arm->target_mutex->lock();
      __arms.right.arm->target_mutex->lock();
      __arms.left.target->trajec = RefPtr<jaco_trajec_t>( trajec_l );
      __arms.right.target->trajec = RefPtr<jaco_trajec_t>( trajec_r );
      // update target.
      // set target->pos accordingly. This makes final-checking in goto_thread much easier
      __arms.left.target->pos = trajec_l->back();
      __arms.right.target->pos = trajec_r->back();
      __arms.left.target->trajec_state=TRAJEC_READY;
      __arms.right.target->trajec_state=TRAJEC_READY;
      __arms.left.arm->target_mutex->unlock();
      __arms.right.arm->target_mutex->unlock();

      return true;
    }
  }
#endif

  return false;
}

bool
JacoBimanualOpenraveThread::_solve_multi_ik(vector<float> &left, vector<float> &right)
{
#ifndef HAVE_OPENRAVE
  return false;
#else
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
    robot->SetActiveManipulator(__arms.left.manip);
    robot->SetActiveDOFs(__arms.left.manip->GetArmIndices());
    __planner_env.robot->set_target_euler(EULER_ZXZ,
                                          __arms.left.target->pos.at(0), __arms.left.target->pos.at(1), __arms.left.target->pos.at(2),
                                          __arms.left.target->pos.at(3), __arms.left.target->pos.at(4), __arms.left.target->pos.at(5));
    IkParameterization param = __planner_env.robot->get_target().ikparam;
    __arms.left.manip->FindIKSolutions(param, solutions_l, IKFO_CheckEnvCollisions);
    if( solutions_l.empty() ) {
      logger->log_warn(name(), "No IK solutions found for left arm");
      return false;
    } else {
      logger->log_debug(name(), "IK solution found for left arm");
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
    robot->SetActiveManipulator(__arms.right.manip);
    robot->SetActiveDOFs(__arms.right.manip->GetArmIndices());
    __planner_env.robot->set_target_euler(EULER_ZXZ,
                                          __arms.right.target->pos.at(0), __arms.right.target->pos.at(1), __arms.right.target->pos.at(2),
                                          __arms.right.target->pos.at(3), __arms.right.target->pos.at(4), __arms.right.target->pos.at(5));
    param = __planner_env.robot->get_target().ikparam;
    __arms.right.manip->FindIKSolutions(param, solutions_r, IKFO_CheckEnvCollisions);
    if( solutions_r.empty() ) {
      logger->log_warn(name(), "No IK solutions found for right arm");
      return false;
    } else {
      logger->log_debug(name(), "IK solution found for right arm");
    }
  } // robot state-saver destroyed

  // restore kinbody states
  for( vector<KinBody::KinBodyStateSaver>::iterator s=statesaver.begin(); s!=statesaver.end(); ++s ) {
    (*s).Restore();
  }

  // finally find the closest solutions without collision and store them
  bool solution_found = false;
  {
    // save state of robot
    RobotBase::RobotStateSaver robot_saver(robot);
    vector< vector<dReal> >::iterator sol_l, sol_r;

    float dist = 100.f;
    vector<dReal> cur_l, cur_r;
    vector<dReal> diff_l, diff_r;
    __arms.left.manip->GetArmDOFValues(cur_l);
    __arms.right.manip->GetArmDOFValues(cur_r);

    // try each combination to find closest non-colliding
    for( sol_l=solutions_l.begin(); sol_l!=solutions_l.end(); ++sol_l ) {
      for( sol_r=solutions_r.begin(); sol_r!=solutions_r.end(); ++sol_r ) {
        // set joints for robot model
        robot->SetDOFValues(*sol_l, 1, __arms.left.manip->GetArmIndices());
        robot->SetDOFValues(*sol_r, 1, __arms.right.manip->GetArmIndices());

        // check for collisions
        if( !robot->CheckSelfCollision() && !robot->GetEnv()->CheckCollision(robot) ) {
          //logger->log_debug(name(), "Collision-free solution found!");
          // calculate distance
          float dist_l = 0.f;
          float dist_r = 0.f;
          diff_l = cur_l;
          diff_r = cur_r;
          robot->SubtractDOFValues(diff_l, (*sol_l), __arms.left.manip->GetArmIndices());
          robot->SubtractDOFValues(diff_r, (*sol_r), __arms.right.manip->GetArmIndices());
          for(unsigned int i=0; i<diff_l.size(); ++i) {
            dist_l += fabs(diff_l[i]);
            // use cur+diff instead of sol, to have better angles
            // for circular joints. Otherwise planner might have problems
            (*sol_l)[i] = cur_l[i] - diff_l[i];
          }
          //logger->log_debug(name(), "Distance left: %f", dist_l);
          for(unsigned int i=0; i<diff_r.size(); ++i) {
            dist_r += fabs(diff_r[i]);
            (*sol_r)[i] = cur_r[i] - diff_r[i];
          }
          //logger->log_debug(name(), "Distance right: %f", dist_r);

          if( dist_l+dist_r < dist ) {
            //logger->log_debug(name(), "Dist %f is closer that previous one (%f). Take this!", dist_l+dist_r, dist);
            dist = dist_l + dist_r;
            solution_found = true;
            left.clear();
            right.clear();
            __planner_env.manip->set_angles(*sol_l);
            __planner_env.manip->get_angles_device(left);
            __planner_env.manip->set_angles(*sol_r);
            __planner_env.manip->get_angles_device(right);
          }
        } else {
          //logger->log_debug(name(), "Skipping solution because of collision!");
        }
      }
    }
  } // robot state-saver destroyed


  return solution_found;
#endif
}
