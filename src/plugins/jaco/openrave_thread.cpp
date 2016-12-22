
/***************************************************************************
 *  openrave_thread.cpp - Jaco plugin OpenRAVE Thread for single arm
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

#include "openrave_thread.h"
#include "arm.h"
#include "types.h"

#include <interfaces/JacoInterface.h>
#include <core/threading/mutex.h>
#include <utils/math/angle.h>

#include <cmath>
#include <stdio.h>
#include <cstring>
#include <unistd.h>

#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/environment.h>
 #include <plugins/openrave/robot.h>
 #include <plugins/openrave/manipulator.h>
 #include <plugins/openrave/manipulators/kinova_jaco.h>

 using namespace OpenRAVE;
#endif

using namespace fawkes;
using namespace std;

/** @class JacoOpenraveThread "openrave_thread.h"
 * Jaco Arm thread for single-arm setup, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param name thread name
 * @param arm pointer to jaco_arm_t struct, to be used in this thread
 * @param load_robot decide if this thread should load a robot. This should only
 *  be set to "true" if a separate OpenRaveRobot should be loaded (e.g. not the
 *  case when using 1 robot with 2 manipulators!)
 */
JacoOpenraveThread::JacoOpenraveThread(const char *name, jaco_arm_t* arm, bool load_robot)
  : JacoOpenraveBaseThread(name)
{
  __arm = arm;
  __load_robot = load_robot;
#ifdef HAVE_OPENRAVE
  __planner_env.env   = NULL;
  __planner_env.robot = NULL;
  __planner_env.manip = NULL;

  __plotted_current = false;
#endif
}

/** Get additional config entries. */
void
JacoOpenraveThread::_init()
{
  switch( __arm->config ) {
    case CONFIG_SINGLE:
      __cfg_manipname = config->get_string("/hardware/jaco/openrave/manipname/single");
      break;

    case CONFIG_LEFT:
      __cfg_manipname = config->get_string("/hardware/jaco/openrave/manipname/dual_left");
      break;

    case CONFIG_RIGHT:
      __cfg_manipname = config->get_string("/hardware/jaco/openrave/manipname/dual_right");
      break;

    default:
      throw fawkes::Exception("Could not read manipname from config.");
      break;
  }
}

/** Load the robot into the environment. */
void
JacoOpenraveThread::_load_robot()
{
#ifdef HAVE_OPENRAVE

  if(__load_robot) {
    __cfg_OR_robot_file    = config->get_string("/hardware/jaco/openrave/robot_file");

    try {
      __viewer_env.robot = openrave->add_robot(__cfg_OR_robot_file, false);
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

      openrave->get_environment()->load_IK_solver(__viewer_env.robot, OpenRAVE::IKP_Transform6D);

    } catch (Exception& e) {
      finalize();
      throw;
    }

  } else {
    // robot was not loaded by this thread. So get them from openrave-environment now
    try {
      __viewer_env.robot = openrave->get_active_robot();
      __viewer_env.manip = __viewer_env.robot->get_manipulator()->copy();
    } catch (Exception& e) {
      throw fawkes::Exception("%s: Could not get robot '%s' from openrave environment. (Error: %s)", name(), __cfg_OR_robot_file.c_str(), e.what_no_backtrace());
    }
  }

#endif //HAVE_OPENRAVE
}

/** Get pointers to the robot and manipulator in the viewer_env, and
 * clone the environment.
 */
void
JacoOpenraveThread::_post_init()
{
#ifdef HAVE_OPENRAVE
  while( !__robot ) {
    __robot = __viewer_env.robot->get_robot_ptr();
    usleep(100);
  }
  while( !__manip ) {
    EnvironmentMutex::scoped_lock lock(__viewer_env.env->get_env_ptr()->GetMutex());
    __manip = __robot->SetActiveManipulator(__cfg_manipname);
    usleep(100);
  }

  // create cloned environment for planning
  logger->log_debug(name(), "Clone environment for planning");
  openrave->clone(__planner_env.env, __planner_env.robot, __planner_env.manip);

  if( !__planner_env.env || !__planner_env.robot || !__planner_env.manip) {
    throw fawkes::Exception("Could not clone properly, received a NULL pointer");
  }

  // set name of environment
  switch( __arm->config ) {
    case CONFIG_SINGLE:
      __planner_env.env->set_name("Planner");
      break;

    case CONFIG_LEFT:
      __planner_env.env->set_name("Planner_Left");
      break;

    case CONFIG_RIGHT:
      __planner_env.env->set_name("Planner_Right");
      break;
  }

  // set active manipulator in planning environment
  {
    EnvironmentMutex::scoped_lock lock(__planner_env.env->get_env_ptr()->GetMutex());
    RobotBase::ManipulatorPtr manip = __planner_env.robot->get_robot_ptr()->SetActiveManipulator(__cfg_manipname);
    __planner_env.robot->get_robot_ptr()->SetActiveDOFs(manip->GetArmIndices());
  }

  // Get chain of links from arm base to manipulator in viewer_env. Used for plotting joints
  __robot->GetChain( __manip->GetBase()->GetIndex(), __manip->GetEndEffector()->GetIndex(), __links);

#endif //HAVE_OPENRAVE
}

void
JacoOpenraveThread::finalize() {
  __arm = NULL;
#ifdef HAVE_OPENRAVE
  if( __load_robot )
    openrave->set_active_robot( NULL );

  __planner_env.robot = NULL;
  __planner_env.manip = NULL;
  __planner_env.env = NULL;

  JacoOpenraveBaseThread::finalize();
#endif
}

/** Mani loop.
 * It iterates over the target_queue to find the first target that needs
 * trajectory planning. This can be done if it is the first target,
 * or if the previous target has a known final configuration, which can
 * be used as the current starting configuration.
 * The result is stored in the struct of the current target, which can
 * then be processed by the goto_thread
 *
 * @see JacoGotoThread#loop to see how goto_thread processes the queue
 */
void
JacoOpenraveThread::loop()
{
#ifndef HAVE_OPENRAVE
    usleep(30e3);
#else
  if( __arm == NULL || __arm->arm == NULL ) {
    usleep(30e3);
    return;
  }

  __planning_mutex->lock();
  RefPtr<jaco_target_t> to;
  // get first target with type TARGET_TRAJEC that needs a planner
  __arm->target_mutex->lock();
  jaco_target_queue_t::iterator it;
  for( it=__arm->target_queue->begin(); it!=__arm->target_queue->end(); ++it ) {
    if( (*it)->trajec_state==TRAJEC_WAITING && !(*it)->coord) {
      // have found a new target for path planning!
      to = *it;
      break;
    }
  }

  if( to ) {
    // Check if there is a prior target that can be usd as the starting position in planning.
    //  The only target-types that can be used for that are those that contain joint positions,
    //  i.e. TARGET_ANGULAR and TARGET_TRAJEC
    RefPtr<jaco_target_t> from;
    while( it!=__arm->target_queue->begin() ) {
      --it;
      if( (*it)->trajec_state==TRAJEC_READY || (*it)->trajec_state==TRAJEC_EXECUTING ) {
        from = RefPtr<jaco_target_t>(new jaco_target_t());
        from->pos = (*it)->trajec->back();
        break;
      } else if( (*it)->trajec_state==TRAJEC_SKIP && (*it)->type == TARGET_ANGULAR ) {
        from = *it;
        break;
      } else if( !((*it)->type==TARGET_GRIPPER) ) {
        // A previous target has unknown final configuration. Cannot plan for our target yet. Abort.
        //  TARGET_GRIPPER would be the only one we could skip without problems.
        __arm->target_mutex->unlock();
        usleep(30e3);
        return;
      }
    }
    __arm->target_mutex->unlock();

    // if there was no prior target that can be used as a starting position, create one
    if( !from ) {
      from = RefPtr<jaco_target_t>(new jaco_target_t());
      __arm->arm->get_joints(from->pos);
    }

    // run planner
    _plan_path(from, to);
    __planning_mutex->unlock();

  } else {
    __arm->target_mutex->unlock();
    __planning_mutex->unlock();
    usleep(30e3); // TODO: make this configurable
  }
#endif
}

void
JacoOpenraveThread::update_openrave()
{
#ifndef HAVE_OPENRAVE
  return;
#else
  if( __arm == NULL || __arm->iface == NULL || __robot == NULL || __manip == NULL )
    return;

  try {
    __joints.clear();
    __joints.push_back(__arm->iface->joints(0));
    __joints.push_back(__arm->iface->joints(1));
    __joints.push_back(__arm->iface->joints(2));
    __joints.push_back(__arm->iface->joints(3));
    __joints.push_back(__arm->iface->joints(4));
    __joints.push_back(__arm->iface->joints(5));

    // get target IK values in openrave format
    __viewer_env.manip->set_angles_device(__joints);
    __viewer_env.manip->get_angles(__joints);

    {
      EnvironmentMutex::scoped_lock lock(__viewer_env.env->get_env_ptr()->GetMutex());
      __robot->SetDOFValues(__joints, 1, __manip->GetArmIndices());
    }

    __joints.clear();
    __joints.push_back( deg2rad(__arm->iface->finger1() - 40.f) );
    __joints.push_back( deg2rad(__arm->iface->finger2() - 40.f) );
    __joints.push_back( deg2rad(__arm->iface->finger3() - 40.f)) ;
    {
      EnvironmentMutex::scoped_lock lock(__viewer_env.env->get_env_ptr()->GetMutex());
      __robot->SetDOFValues(__joints, 1, __manip->GetGripperIndices());
    }

    if( __plot_current ) {
      EnvironmentMutex::scoped_lock lock(__viewer_env.env->get_env_ptr()->GetMutex());

      if( !__plotted_current ) {
        // new plotting command. Erase previous plot
        __graph_current.clear();
      }

      if( __cfg_OR_plot_cur_manip ) {
        OpenRAVE::Vector color(0.f, 1.f, 0.f, 1.f);
        const OpenRAVE::Vector &trans = __manip->GetEndEffectorTransform().trans;
        float transa[4] = { (float)trans.x, (float)trans.y, (float)trans.z, (float)trans.w };
        __graph_current.push_back( __viewer_env.env->get_env_ptr()->plot3(transa,1, 0, 2.f, color));
      }

      if( __cfg_OR_plot_cur_joints ) {
        OpenRAVE::Vector color(0.2f, 1.f, 0.f, 1.f);
        for(unsigned int i=0; i<__links.size(); ++i) {
          const OpenRAVE::Vector &trans = __links[i]->GetTransform().trans;
          float transa[4] = { (float)trans.x, (float)trans.y, (float)trans.z, (float)trans.w };
          __graph_current.push_back( __viewer_env.env->get_env_ptr()->plot3(transa,1, 0, 2.f, color));
        }
      }
    }
    __plotted_current = __plot_current;

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
#endif
}

/** Solve IK and add target to the queue.
 *
 * The IK is solved, ignoring collisions of the end-effector with the environment.
 * We do this to generally decide if IK is generally solvable. Collision checking
 * is done in a later step in JacoOpenraveThread::_plan_path .
 *
 * If IK is solvable, the target is enqueued in the target_queue.
 *
 * @param x x-coordinate of target position
 * @param y y-coordinate of target position
 * @param z z-coordinate of target position
 * @param e1 1st euler rotation of target orientation
 * @param e2 2nd euler rotation of target orientation
 * @param e3 3rd euler rotation of target orientation
 * @param plan decide if we want to plan a trajectory for this or not
 * @return "true", if IK could be solved. "false" otherwise
 */
bool
JacoOpenraveThread::add_target(float x, float y, float z, float e1, float e2, float e3, bool plan)
{
  bool solvable = false;  // need to define it here outside the ifdef-scope

#ifdef HAVE_OPENRAVE
  try {
    // update planner params; set correct DOF and stuff
    __planner_env.robot->get_planner_params();

    if( plan ) {
      // get IK from openrave. Ignore collisions with env though, as this is only for IK check and env might change at the
      //  time we start planning. There will be separate IK checks though for planning!
      __planner_env.robot->enable_ik_comparison(false);
      solvable = __planner_env.robot->set_target_euler(EULER_ZXZ, x, y, z, e1, e2, e3, IKFO_IgnoreEndEffectorEnvCollisions);

      if( solvable ) {
        // add this to the target queue for planning
        logger->log_debug(name(), "Adding to target_queue for later planning");

        // create new target for the queue
        RefPtr<jaco_target_t> target(new jaco_target_t());
        target->type = TARGET_CARTESIAN;
        target->trajec_state = TRAJEC_WAITING;
        target->coord=false;
        target->pos.push_back(x);
        target->pos.push_back(y);
        target->pos.push_back(z);
        target->pos.push_back(e1);
        target->pos.push_back(e2);
        target->pos.push_back(e3);

        __arm->target_mutex->lock();
        __arm->target_queue->push_back(target);
        __arm->target_mutex->unlock();
      } else {
        logger->log_warn(name(), "No IK solution found for target.");
      }

    } else {
      // don't plan, consider this the final configuration

      // get IK from openrave. Do not ignore collisions this time, because we skip planning
      //  and go straight to this configuration!
      solvable = __planner_env.robot->set_target_euler(EULER_ZXZ, x, y, z, e1, e2, e3);

      if( solvable ) {
        logger->log_debug(name(), "Skip planning, add this as TARGET_ANGULAR");

        // create new target for the queue
        RefPtr<jaco_target_t> target(new jaco_target_t());
        target->type = TARGET_ANGULAR;
        target->trajec_state = TRAJEC_SKIP;
        target->coord=false;
        // get target IK values
        __planner_env.robot->get_target().manip->get_angles_device(target->pos);

        __arm->target_mutex->lock();
        __arm->target_queue->push_back(target);
        __arm->target_mutex->unlock();
      } else {
        logger->log_warn(name(), "No IK solution found for target.");
      }
    }

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
#endif

  return solvable;
}


/** Add target joint values to the queue.
 *
 * Use this method with caution, as for now there are no checks for validity
 * of the target joint values. This will be added soon.
 * Collision checking with the environment is done in a later step
 * in JacoOpenraveThread::_plan_path .
 *
 * @param j1 target angle of 1st joint
 * @param j2 target angle of 2nd joint
 * @param j3 target angle of 3rd joint
 * @param j4 target angle of 4th joint
 * @param j5 target angle of 5th joint
 * @param j6 target angle of 6th joint
 * @param plan decide if we want to plan a trajectory for this or not
 * @return "true", if the target joints are valid and not in self-collision,
 *  "false" otherwise.
 *  CAUTION: Self-collision is not checked yet, this feature will be added soon.
 */
bool
JacoOpenraveThread::add_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, bool plan)
{
  bool joints_valid = false;  // need to define it here outside the ifdef-scope

#ifdef HAVE_OPENRAVE
  try {
    // update planner params; set correct DOF and stuff
    __planner_env.robot->get_planner_params();

    //TODO: need some kind cheking for self-collision, i.e. if the joint values are "valid".
    // For now expect the user to know what he does, when he sets joint angles directly
    joints_valid = true;

    // create new target for the queue
    RefPtr<jaco_target_t> target(new jaco_target_t());
    target->type = TARGET_ANGULAR;
    target->trajec_state = plan ? TRAJEC_WAITING : TRAJEC_SKIP;
    target->coord=false;
    target->pos.push_back(j1);
    target->pos.push_back(j2);
    target->pos.push_back(j3);
    target->pos.push_back(j4);
    target->pos.push_back(j5);
    target->pos.push_back(j6);

    __arm->target_mutex->lock();
    __arm->target_queue->push_back(target);
    __arm->target_mutex->unlock();

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
#endif

  return joints_valid;
}


/** Flush the target_queue and add this one.
 * see JacoOpenraveThread#add_target for that.
 *
 * @param x x-coordinate of target position
 * @param y y-coordinate of target position
 * @param z z-coordinate of target position
 * @param e1 1st euler rotation of target orientation
 * @param e2 2nd euler rotation of target orientation
 * @param e3 3rd euler rotation of target orientation
 * @param plan decide if we want to plan a trajectory for this or not
 * @return "true", if IK could be solved. "false" otherwise
 */
bool
JacoOpenraveThread::set_target(float x, float y, float z, float e1, float e2, float e3, bool plan)
{
  __arm->target_mutex->lock();
  __arm->target_queue->clear();
  __arm->target_mutex->unlock();
  return add_target(x, y, z, e1, e2, e3, plan);
}


/** Flush the target_queue and add this one.
 * see JacoOpenraveThread#add_target_ang for that.
 *
 * @param j1 target angle of 1st joint
 * @param j2 target angle of 2nd joint
 * @param j3 target angle of 3rd joint
 * @param j4 target angle of 4th joint
 * @param j5 target angle of 5th joint
 * @param j6 target angle of 6th joint
 * @param plan decide if we want to plan a trajectory for this or not
 * @return "true", if IK could be solved. "false" otherwise
 */
bool
JacoOpenraveThread::set_target_ang(float j1, float j2, float j3, float j4, float j5, float j6, bool plan)
{
  __arm->target_mutex->lock();
  __arm->target_queue->clear();
  __arm->target_mutex->unlock();
  return add_target_ang(j1, j2, j3, j4, j5, j6, plan);
}


void
JacoOpenraveThread::_plan_path(RefPtr<jaco_target_t> &from, RefPtr<jaco_target_t> &to)
{
#ifdef HAVE_OPENRAVE
  // update state of the trajectory
  __arm->target_mutex->lock();
  to->trajec_state = TRAJEC_PLANNING;
  __arm->target_mutex->unlock();

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

  // restore robot state
  {
    EnvironmentMutex::scoped_lock lock(__planner_env.env->get_env_ptr()->GetMutex());

    // Set active manipulator and active DOFs (need for planner and IK solver!)
    RobotBase::ManipulatorPtr manip = __planner_env.robot->get_robot_ptr()->SetActiveManipulator(__cfg_manipname);
    __planner_env.robot->get_robot_ptr()->SetActiveDOFs(manip->GetArmIndices());

    // update robot state with attached objects
    {
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
        logger->log_debug(name(), "compare _robotlinkname '%s' with our manip link '%s'",
                          (*it)->_robotlinkname.c_str(), manip->GetEndEffector()->GetName().c_str());
        if( (*it)->_robotlinkname == manip->GetEndEffector()->GetName() ) {
          logger->log_debug(name(), "attach '%s'!", (*it)->_grabbedname.c_str());
          __planner_env.robot->attach_object((*it)->_grabbedname.c_str(), __planner_env.env, __cfg_manipname.c_str());
        }
      }
    }
  }

  // Set target point for planner. Check again for IK, avoiding collisions with the environment
  //logger->log_debug(name(), "setting target %f %f %f %f %f %f",
  //                  to->pos.at(0), to->pos.at(1), to->pos.at(2), to->pos.at(3), to->pos.at(4), to->pos.at(5));
  __planner_env.robot->enable_ik_comparison(true);
  if( to->type == TARGET_CARTESIAN ) {
    if( !__planner_env.robot->set_target_euler(EULER_ZXZ, to->pos.at(0), to->pos.at(1), to->pos.at(2), to->pos.at(3), to->pos.at(4), to->pos.at(5)) ) {
      logger->log_warn(name(), "Planning failed, second IK check failed");
      __arm->target_mutex->lock();
      to->trajec_state = TRAJEC_PLANNING_ERROR;
      __arm->target_mutex->unlock();
      return;

    } else {
      // set target angles. This changes the internal target type to ANGLES (see openrave/robot.*)
      //  and will use BaseManipulation's MoveActiveJoints. Otherwise it will use MoveToHandPosition,
      //  which does not have the filtering of IK solutions for the closest one as we have.
      vector<float> target;
      __planner_env.robot->get_target().manip->get_angles(target);
      __planner_env.robot->set_target_angles(target);
    }

  } else {
    vector<float> target;
    //TODO: need some kind cheking for env-collision, i.e. if the target is colllision-free.
    // For now expect the user to know what he does, when he sets joint angles directly
    __planner_env.robot->get_target().manip->set_angles_device(to->pos);

    __planner_env.robot->get_target().manip->get_angles(target);
    __planner_env.robot->set_target_angles(target);
  }

  // Set starting point for planner
  //logger->log_debug(name(), "setting start %f %f %f %f %f %f",
  //                  from->pos.at(0), from->pos.at(1), from->pos.at(2), from->pos.at(3), from->pos.at(4), from->pos.at(5));
  __planner_env.manip->set_angles_device(from->pos);

  // Set planning parameters
  __planner_env.robot->set_target_plannerparams(__plannerparams);

  // Run planner
  try {
    __planner_env.env->run_planner(__planner_env.robot, __cfg_OR_sampling);
  } catch (fawkes::Exception &e) {
    logger->log_warn(name(), "Planning failed: %s", e.what_no_backtrace());
    // TODO: better handling!
    // for now just skip planning, so the target_queue can be processed
    __arm->target_mutex->lock();
    //to->type = TARGET_ANGULAR;
    to->trajec_state = TRAJEC_PLANNING_ERROR;
    __arm->target_mutex->unlock();
    return;
  }

  // add trajectory to queue
  //logger->log_debug(name(), "plan successful, adding to queue");
  __arm->trajec_mutex->lock();
  // we can do the following becaouse get_trajectory_device() returns a new object, thus
  //  can be safely deleted by RefPtr auto-deletion
  to->trajec = RefPtr<jaco_trajec_t>( __planner_env.robot->get_trajectory_device() );
  __arm->trajec_mutex->unlock();

  // update target.
  __arm->target_mutex->lock();
  //change target type to ANGULAR and set target->pos accordingly. This makes final-checking
  // in goto_thread much easier
  to->type = TARGET_ANGULAR;
  to->pos = to->trajec->back();
  // update trajectory state
  to->trajec_state = TRAJEC_READY;
  __arm->target_mutex->unlock();
#endif
}



/** Plot the first target of the queue in the viewer_env */
void
JacoOpenraveThread::plot_first()
{
#ifdef HAVE_OPENRAVE
  if( !__cfg_OR_use_viewer || (!__cfg_OR_plot_traj_manip && !__cfg_OR_plot_traj_joints))
    return;

  __graph_handle.clear();

  // check if there is a target to be plotted
  __arm->target_mutex->lock();
  if( __arm->target_queue->empty() ) {
    __arm->target_mutex->unlock();
    return;
  }

  // get RefPtr to first target in queue
  RefPtr<jaco_target_t> target = __arm->target_queue->front();
  __arm->target_mutex->unlock();


  // only plot trajectories
  if( target->trajec_state != TRAJEC_READY && target->trajec_state != TRAJEC_EXECUTING )
    return;

  // plot the trajectory (if possible)
  __arm->trajec_mutex->lock();
  if( !target->trajec ) {
    __arm->trajec_mutex->unlock();
    return;
  }

  // remove all GraphHandlerPtr and currently drawn plots
  __graph_handle.clear();
  {
    EnvironmentMutex::scoped_lock lock(__viewer_env.env->get_env_ptr()->GetMutex());

    // save the state, do not modifiy currently active robot!
    RobotBasePtr tmp_robot = __viewer_env.robot->get_robot_ptr();
    RobotBase::RobotStateSaver saver(tmp_robot);

    std::vector<dReal> joints;
    OpenRaveManipulatorPtr manip = __viewer_env.manip->copy();

    OpenRAVE::Vector color_m(__arm->trajec_color[0],
                             __arm->trajec_color[1],
                             __arm->trajec_color[2],
                             __arm->trajec_color[3]);
    OpenRAVE::Vector color_j(__arm->trajec_color[0] / 1.4f,
                             0.2f,
                             __arm->trajec_color[2] / 1.4f,
                             __arm->trajec_color[3] / 1.4f);

    for(jaco_trajec_t::iterator it = target->trajec->begin(); it!=target->trajec->end(); ++it) {
      manip->set_angles_device((*it));
      manip->get_angles(joints);

      tmp_robot->SetDOFValues(joints, 1, __manip->GetArmIndices());

      if( __cfg_OR_plot_traj_manip ) {
        const OpenRAVE::Vector &trans = __manip->GetEndEffectorTransform().trans;
        float transa[4] = { (float)trans.x, (float)trans.y, (float)trans.z, (float)trans.w };
        __graph_handle.push_back( __viewer_env.env->get_env_ptr()->plot3(transa,1, 0, 3.f, color_m));
      }

      if( __cfg_OR_plot_traj_joints ) {
        for(unsigned int i=0; i<__links.size(); ++i) {
          const OpenRAVE::Vector &trans = __links[i]->GetTransform().trans;
          float transa[4] = { (float)trans.x, (float)trans.y, (float)trans.z, (float)trans.w };
          __graph_handle.push_back( __viewer_env.env->get_env_ptr()->plot3(transa,1, 0, 3.f, color_j));
        }
      }
    }
  } // robot state is restored

  __arm->trajec_mutex->unlock();

#endif //HAVE_OPENRAVE
}
