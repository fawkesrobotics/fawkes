
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
  arms_.left.arm = arms->left;
  arms_.right.arm = arms->right;
#ifdef HAVE_OPENRAVE
  planner_env_.env   = NULL;
  planner_env_.robot = NULL;
  planner_env_.manip = NULL;
#endif

  __constrained = false;
}

void
JacoBimanualOpenraveThread::_init()
{
#ifdef HAVE_OPENRAVE
  arms_.left.manipname  = config->get_string("/hardware/jaco/openrave/manipname/dual_left");
  arms_.right.manipname = config->get_string("/hardware/jaco/openrave/manipname/dual_right");
#endif
}

void
JacoBimanualOpenraveThread::_load_robot()
{
#ifdef HAVE_OPENRAVE
  cfg_OR_robot_file_  = config->get_string("/hardware/jaco/openrave/robot_dual_file");

  try {
    //viewer_env_.robot = openrave->add_robot(cfg_OR_robot_file_, false);
    // manually add robot; the automatic needs to be altered
    viewer_env_.robot = new OpenRaveRobot(logger);
    viewer_env_.robot->load(cfg_OR_robot_file_, viewer_env_.env);
    viewer_env_.env->add_robot(viewer_env_.robot);
    viewer_env_.robot->set_ready();
    openrave->set_active_robot(viewer_env_.robot);
  } catch (Exception& e) {
    throw fawkes::Exception("Could not add robot '%s' to openrave environment. (Error: %s)", cfg_OR_robot_file_.c_str(), e.what_no_backtrace());
  }

  try {
    viewer_env_.manip = new OpenRaveManipulatorKinovaJaco(6, 6);
    viewer_env_.manip->add_motor(0,0);
    viewer_env_.manip->add_motor(1,1);
    viewer_env_.manip->add_motor(2,2);
    viewer_env_.manip->add_motor(3,3);
    viewer_env_.manip->add_motor(4,4);
    viewer_env_.manip->add_motor(5,5);

    // Set manipulator and offsets.
    openrave->set_manipulator(viewer_env_.robot, viewer_env_.manip, 0.f, 0.f, 0.f);

    EnvironmentMutex::scoped_lock lock(viewer_env_.env->get_env_ptr()->GetMutex());

    arms_.right.manip = viewer_env_.robot->get_robot_ptr()->SetActiveManipulator(arms_.right.manipname);
    if( cfg_OR_auto_load_ik_ ) {
      logger->log_debug(name(), "load IK for right arm");
      viewer_env_.env->load_IK_solver(viewer_env_.robot, OpenRAVE::IKP_Transform6D);
    }

    arms_.left.manip = viewer_env_.robot->get_robot_ptr()->SetActiveManipulator(arms_.left.manipname);
    if( cfg_OR_auto_load_ik_ ) {
      logger->log_debug(name(), "load IK for left arm");
      viewer_env_.env->load_IK_solver(viewer_env_.robot, OpenRAVE::IKP_Transform6D);
    }

  } catch (Exception& e) {
    finalize();
    throw;
  }

  // create cloned environment for planning
  logger->log_debug(name(), "Clone environment for planning");
  openrave->clone(planner_env_.env, planner_env_.robot, planner_env_.manip);

  if( !planner_env_.env || !planner_env_.robot || !planner_env_.manip) {
    throw fawkes::Exception("Could not clone properly, received a NULL pointer");
  }

  // set name of env
  planner_env_.env->set_name("Planner_Bimanual");

  // set manips to those of planner env
  arms_.right.manip = planner_env_.robot->get_robot_ptr()->SetActiveManipulator(arms_.right.manipname);
  arms_.left.manip = planner_env_.robot->get_robot_ptr()->SetActiveManipulator(arms_.left.manipname);

  // initial modules for dualmanipulation
  _init_dualmanipulation();
#endif
}


void
JacoBimanualOpenraveThread::_init_dualmanipulation()
{
#ifdef HAVE_OPENRAVE
  // load dualmanipulation module
  EnvironmentMutex::scoped_lock lock(planner_env_.env->get_env_ptr()->GetMutex());
  mod_dualmanip_ = RaveCreateModule(planner_env_.env->get_env_ptr(), "dualmanipulation");
  planner_env_.env->get_env_ptr()->Add( mod_dualmanip_, true, planner_env_.robot->get_robot_ptr()->GetName());

  // load MultiManipIkSolver stuff
  // Get all the links that are affecte by left/right manipulator
  vector<int> arm_idx_l = arms_.left.manip->GetArmIndices();
  vector<int> arm_idx_r = arms_.right.manip->GetArmIndices();
  vector<int> grp_idx = arms_.left.manip->GetGripperIndices();
  arm_idx_l.reserve( arm_idx_l.size() + grp_idx.size() );
  arm_idx_l.insert( arm_idx_l.end(), grp_idx.begin(), grp_idx.end() );
  grp_idx = arms_.right.manip->GetGripperIndices();
  arm_idx_r.reserve( arm_idx_r.size() + grp_idx.size() );
  arm_idx_r.insert( arm_idx_r.end(), grp_idx.begin(), grp_idx.end() );

  RobotBasePtr robot = planner_env_.robot->get_robot_ptr();
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
  arms_.left.arm = NULL;
  arms_.right.arm = NULL;
#ifdef HAVE_OPENRAVE
  openrave->set_active_robot( NULL );

  planner_env_.env->get_env_ptr()->Remove( mod_dualmanip_ );
  planner_env_.robot = NULL;
  planner_env_.manip = NULL;
  planner_env_.env = NULL;
#endif

  JacoOpenraveBaseThread::finalize();
}


void
JacoBimanualOpenraveThread::loop()
{
#ifndef HAVE_OPENRAVE
  usleep(30e3);
#else
  if( arms_.left.arm == NULL || arms_.right.arm == NULL ) {
    usleep(30e3);
    return;
  }

  // get first target in queues
  arms_.left.arm->target_mutex->lock();
  arms_.right.arm->target_mutex->lock();
  if( !arms_.left.arm->target_queue->empty() && !arms_.right.arm->target_queue->empty() ) {
    arms_.left.target  = arms_.left.arm->target_queue->front();
    arms_.right.target = arms_.right.arm->target_queue->front();
  }
  arms_.left.arm->target_mutex->unlock();
  arms_.right.arm->target_mutex->unlock();

  if( !arms_.left.target || !arms_.right.target
   || !arms_.left.target->coord || !arms_.right.target->coord
   || arms_.left.target->trajec_state !=TRAJEC_WAITING
   || arms_.right.target->trajec_state!=TRAJEC_WAITING ) {
    //no new target in queue, or target is not meant for coordinated bimanual manipulation
    usleep(30e3);
    return;
  }

  // copy environment first
  _copy_env();

  // get suiting IK solutions
  vector<float> sol_l, sol_r;
  bool solvable = _solve_multi_ik(sol_l, sol_r);
  arms_.left.arm->target_mutex->lock();
  arms_.right.arm->target_mutex->lock();
  if( !solvable ) {
    arms_.left.target->trajec_state=TRAJEC_IK_ERROR;
    arms_.right.target->trajec_state=TRAJEC_IK_ERROR;
    arms_.left.arm->target_mutex->unlock();
    arms_.right.arm->target_mutex->unlock();
    usleep(30e3);
    return;
  } else {
    arms_.left.target->type=TARGET_ANGULAR;
    arms_.left.target->pos = sol_l;
    arms_.right.target->type=TARGET_ANGULAR;
    arms_.right.target->pos = sol_r;
    arms_.left.arm->target_mutex->unlock();
    arms_.right.arm->target_mutex->unlock();

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

  arms_.left.arm->target_mutex->lock();
  arms_.right.arm->target_mutex->lock();
  arms_.left.arm->target_queue->push_back(target_l);
  arms_.right.arm->target_queue->push_back(target_r);
  arms_.left.arm->target_mutex->unlock();
  arms_.right.arm->target_mutex->unlock();

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
  arms_.left.arm->target_mutex->lock();
  arms_.right.arm->target_mutex->lock();
  arms_.left.target->trajec_state=state;
  arms_.right.target->trajec_state=state;
  arms_.left.arm->target_mutex->unlock();
  arms_.right.arm->target_mutex->unlock();
#endif
}

void
JacoBimanualOpenraveThread::_copy_env()
{
#ifdef HAVE_OPENRAVE
  // Update bodies in planner-environment
  // clone robot state, ignoring grabbed bodies
  {
    EnvironmentMutex::scoped_lock view_lock(viewer_env_.env->get_env_ptr()->GetMutex());
    EnvironmentMutex::scoped_lock plan_lock(planner_env_.env->get_env_ptr()->GetMutex());
    planner_env_.robot->get_robot_ptr()->ReleaseAllGrabbed();
    planner_env_.env->delete_all_objects();

    /*
    // Old method. Somehow we encountered problems. OpenRAVE internal bug?
    RobotBase::RobotStateSaver saver(viewer_env_.robot->get_robot_ptr(),
                                     0xffffffff&~KinBody::Save_GrabbedBodies&~KinBody::Save_ActiveManipulator&~KinBody::Save_ActiveDOF);
    saver.Restore( planner_env_.robot->get_robot_ptr() );
    */
    // New method. Simply set the DOF values as they are in viewer_env_
    vector<dReal> dofs;
    viewer_env_.robot->get_robot_ptr()->GetDOFValues(dofs);
    planner_env_.robot->get_robot_ptr()->SetDOFValues(dofs);
  }

  // then clone all objects
  planner_env_.env->clone_objects( viewer_env_.env );

  // update robot state with attached objects
  {
    EnvironmentMutex::scoped_lock lock(planner_env_.env->get_env_ptr()->GetMutex());
    EnvironmentMutex::scoped_lock view_lock(viewer_env_.env->get_env_ptr()->GetMutex());
    /*
    // Old method. Somehow we encountered problems. OpenRAVE internal bug?
    RobotBase::RobotStateSaver saver(viewer_env_.robot->get_robot_ptr(),
                                     KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_GrabbedBodies);
    saver.Restore( planner_env_.robot->get_robot_ptr() );
    */
    // New method. Grab all bodies in planner_env_ that are grabbed in viewer_env_ by this manipulator
    vector<RobotBase::GrabbedInfoPtr> grabbed;
    viewer_env_.robot->get_robot_ptr()->GetGrabbedInfo(grabbed);
    for( vector<RobotBase::GrabbedInfoPtr>::iterator it=grabbed.begin(); it!=grabbed.end(); ++it ) {
      logger->log_debug(name(), "compare _robotlinkname '%s' with our manip links '%s' and '%s'",
                        (*it)->_robotlinkname.c_str(),
                        arms_.left.manip->GetEndEffector()->GetName().c_str(),
                        arms_.right.manip->GetEndEffector()->GetName().c_str());
      if( (*it)->_robotlinkname == arms_.left.manip->GetEndEffector()->GetName() ) {
        logger->log_debug(name(), "attach '%s' to '%s'!", (*it)->_grabbedname.c_str(), arms_.left.manip->GetEndEffector()->GetName().c_str());
        planner_env_.robot->attach_object((*it)->_grabbedname.c_str(), planner_env_.env, arms_.left.manipname.c_str());

      } else if( (*it)->_robotlinkname == arms_.right.manip->GetEndEffector()->GetName() ) {
        logger->log_debug(name(), "attach '%s' to '%s'!", (*it)->_grabbedname.c_str(), arms_.right.manip->GetEndEffector()->GetName().c_str());
        planner_env_.robot->attach_object((*it)->_grabbedname.c_str(), planner_env_.env, arms_.right.manipname.c_str());
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

  EnvironmentMutex::scoped_lock lock(planner_env_.env->get_env_ptr()->GetMutex());

  // Set  active DOFs
  vector<int> dofs   = arms_.left.manip->GetArmIndices();
  vector<int> dofs_r = arms_.right.manip->GetArmIndices();
  dofs.reserve(dofs.size() + dofs_r.size());
  dofs.insert(dofs.end(), dofs_r.begin(), dofs_r.end());
  planner_env_.robot->get_robot_ptr()->SetActiveDOFs(dofs);

  // setup command for dualmanipulation module
  stringstream cmdin,cmdout;
  cmdin << std::setprecision(numeric_limits<dReal>::digits10+1);
  cmdout << std::setprecision(numeric_limits<dReal>::digits10+1);

  vector<dReal> sol;
  cmdin << "MoveAllJoints goal";
  planner_env_.manip->set_angles_device(arms_.left.target->pos);
  planner_env_.manip->get_angles(sol);
  for(size_t i = 0; i < sol.size(); ++i) {
    cmdin << " " << sol[i];
  }
  planner_env_.manip->set_angles_device(arms_.right.target->pos);
  planner_env_.manip->get_angles(sol);
  for(size_t i = 0; i < sol.size(); ++i) {
    cmdin << " " << sol[i];
  }

  //add additional planner parameters
  if( !plannerparams_.empty() ) {
    cmdin << " " << plannerparams_;
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
    success = mod_dualmanip_->SendCommand(cmdout,cmdin);
  } catch(openrave_exception &e) {
    logger->log_debug(name(), "Planner: dualmanip command failed. Ex:%s", e.what());
  }

  if(!success) {
    logger->log_warn(name(),"Planner: planning failed");
    _set_trajec_state(TRAJEC_PLANNING_ERROR);
    arms_.left.arm->target_mutex->lock();
    arms_.right.arm->target_mutex->lock();
    arms_.left.arm->target_mutex->unlock();
    arms_.right.arm->target_mutex->unlock();
    return false;

  } else {
    //logger->log_debug(name(), "Planner: path planned. cmdout:%s", cmdout.str().c_str());

    // read returned trajectory
    ConfigurationSpecification cfg_spec = planner_env_.robot->get_robot_ptr()->GetActiveConfigurationSpecification();
    TrajectoryBasePtr traj = RaveCreateTrajectory(planner_env_.env->get_env_ptr(), "");
    traj->Init(cfg_spec);
    if( !traj->deserialize(cmdout) ) {
      logger->log_warn(name(), "Planner: Cannot read trajectory data.");
      _set_trajec_state(TRAJEC_PLANNING_ERROR);
      arms_.left.arm->target_mutex->lock();
      arms_.right.arm->target_mutex->lock();
      arms_.left.arm->target_mutex->unlock();
      arms_.right.arm->target_mutex->unlock();
      return false;

    } else {
      // sampling trajectory and setting target trajectory
      jaco_trajec_t* trajec_l = new jaco_trajec_t();
      jaco_trajec_t* trajec_r = new jaco_trajec_t();
      jaco_trajec_point_t p; // point we will add to trajectories
      vector<dReal> tmp_p;
      int arm_dof = cfg_spec.GetDOF() / 2;

      for(dReal time = 0; time <= traj->GetDuration(); time += (dReal)cfg_OR_sampling_) {
        vector<dReal> point;
        traj->Sample(point,time);

        tmp_p = vector<dReal>(point.begin(), point.begin()+arm_dof);
        planner_env_.manip->angles_or_to_device( tmp_p, p);
        trajec_l->push_back(p);

        tmp_p = vector<dReal>(point.begin()+arm_dof, point.begin()+2*arm_dof);
        planner_env_.manip->angles_or_to_device( tmp_p, p);
        trajec_r->push_back(p);
      }

      arms_.left.arm->target_mutex->lock();
      arms_.right.arm->target_mutex->lock();
      arms_.left.target->trajec = RefPtr<jaco_trajec_t>( trajec_l );
      arms_.right.target->trajec = RefPtr<jaco_trajec_t>( trajec_r );
      // update target.
      // set target->pos accordingly. This makes final-checking in goto_thread much easier
      arms_.left.target->pos = trajec_l->back();
      arms_.right.target->pos = trajec_r->back();
      arms_.left.target->trajec_state=TRAJEC_READY;
      arms_.right.target->trajec_state=TRAJEC_READY;
      arms_.left.arm->target_mutex->unlock();
      arms_.right.arm->target_mutex->unlock();

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
  EnvironmentMutex::scoped_lock plan_lock(planner_env_.env->get_env_ptr()->GetMutex());

  // robot ptr for convenienc
  RobotBasePtr robot = planner_env_.robot->get_robot_ptr();

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

    // Find IK solutions for left arm
    // Disable all links of right manipulator
    for( set<KinBody::LinkPtr>::iterator body=links_right_.begin(); body!=links_right_.end(); ++body) {
      (*body)->Enable(false);
    }
    // Enable only grabbed bodies of this manipulator
    for( vector<KinBodyPtr>::iterator body=grabbed.begin(); body!=grabbed.end(); ++body) {
      (*body)->Enable(arms_.left.manip->IsGrabbing(*body));
    }
    // Get Ik Solutions.
    robot->SetActiveManipulator(arms_.left.manip);
    robot->SetActiveDOFs(arms_.left.manip->GetArmIndices());
    planner_env_.robot->set_target_euler(EULER_ZXZ,
                                          arms_.left.target->pos.at(0), arms_.left.target->pos.at(1), arms_.left.target->pos.at(2),
                                          arms_.left.target->pos.at(3), arms_.left.target->pos.at(4), arms_.left.target->pos.at(5));
    IkParameterization param = planner_env_.robot->get_target().ikparam;
    arms_.left.manip->FindIKSolutions(param, solutions_l, IKFO_CheckEnvCollisions);
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
      (*body)->Enable(arms_.right.manip->IsGrabbing(*body));
    }
    // Get Ik Solutions.
    robot->SetActiveManipulator(arms_.right.manip);
    robot->SetActiveDOFs(arms_.right.manip->GetArmIndices());
    planner_env_.robot->set_target_euler(EULER_ZXZ,
                                          arms_.right.target->pos.at(0), arms_.right.target->pos.at(1), arms_.right.target->pos.at(2),
                                          arms_.right.target->pos.at(3), arms_.right.target->pos.at(4), arms_.right.target->pos.at(5));
    param = planner_env_.robot->get_target().ikparam;
    arms_.right.manip->FindIKSolutions(param, solutions_r, IKFO_CheckEnvCollisions);
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
    arms_.left.manip->GetArmDOFValues(cur_l);
    arms_.right.manip->GetArmDOFValues(cur_r);

    // try each combination to find closest non-colliding
    for( sol_l=solutions_l.begin(); sol_l!=solutions_l.end(); ++sol_l ) {
      for( sol_r=solutions_r.begin(); sol_r!=solutions_r.end(); ++sol_r ) {
        // set joints for robot model
        robot->SetDOFValues(*sol_l, 1, arms_.left.manip->GetArmIndices());
        robot->SetDOFValues(*sol_r, 1, arms_.right.manip->GetArmIndices());

        // check for collisions
        if( !robot->CheckSelfCollision() && !robot->GetEnv()->CheckCollision(robot) ) {
          //logger->log_debug(name(), "Collision-free solution found!");
          // calculate distance
          float dist_l = 0.f;
          float dist_r = 0.f;
          diff_l = cur_l;
          diff_r = cur_r;
          robot->SubtractDOFValues(diff_l, (*sol_l), arms_.left.manip->GetArmIndices());
          robot->SubtractDOFValues(diff_r, (*sol_r), arms_.right.manip->GetArmIndices());
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
            planner_env_.manip->set_angles(*sol_l);
            planner_env_.manip->get_angles_device(left);
            planner_env_.manip->set_angles(*sol_r);
            planner_env_.manip->get_angles_device(right);
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
