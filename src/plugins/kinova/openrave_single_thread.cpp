
/***************************************************************************
 *  openrave_single_thread.cpp - Kinova plugin OpenRAVE Thread for single-arm setup
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

#include "openrave_single_thread.h"
#include "arm.h"
#include "types.h"

#include <interfaces/JacoInterface.h>
#include <core/threading/mutex.h>

#include <cmath>
#include <stdio.h>
#include <cstring>

#ifdef HAVE_OPENRAVE
 #include <plugins/openrave/environment.h>
 #include <plugins/openrave/robot.h>
 #include <plugins/openrave/manipulator.h>
 #include <plugins/openrave/manipulators/kinova_jaco.h>

 using namespace OpenRAVE;
#endif

using namespace fawkes;
using namespace std;

/** @class KinovaOpenraveSingleThread "openrave_single_thread.h"
 * Jaco Arm thread for single-arm setup, integrating OpenRAVE
 *
 * @author Bahram Maleki-Fard
 */

/** Constructor.
 * @param thread_name thread name
 */
KinovaOpenraveSingleThread::KinovaOpenraveSingleThread(const char *manipname, bool load_robot)
  : KinovaOpenraveBaseThread("KinovaOpenraveSingleThread")
{
  __arm = NULL;
  __manipname = manipname;
  __load_robot = load_robot;
#ifdef HAVE_OPENRAVE
  __planner_env.env   = NULL;
  __planner_env.robot = NULL;
  __planner_env.manip = NULL;
#endif
}

/** Constructor.
 * @param thread_name thread name
 */
KinovaOpenraveSingleThread::KinovaOpenraveSingleThread(const char *name, const char *manipname, bool load_robot)
  : KinovaOpenraveBaseThread(name)
{
  __arm = NULL;
  __manipname = manipname;
  __load_robot = load_robot;
#ifdef HAVE_OPENRAVE
  __planner_env.env   = NULL;
  __planner_env.robot = NULL;
  __planner_env.manip = NULL;
#endif
}


void
KinovaOpenraveSingleThread::_load_robot()
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

      if( __cfg_OR_auto_load_ik ) {
        openrave->get_environment()->load_IK_solver(__viewer_env.robot, OpenRAVE::IKP_Transform6D);
      }

    } catch (Exception& e) {
      finalize();
      throw;
    }
  }
#endif //HAVE_OPENRAVE
}

void
KinovaOpenraveSingleThread::once()
{
#ifdef HAVE_OPENRAVE
  if(!__load_robot) {
    // robot was not loaded by this thread. So get them from openrave-environment now
    try {
      __viewer_env.robot = openrave->get_active_robot();
      __viewer_env.manip = __viewer_env.robot->get_manipulator(); //TODO: use new(), copy constructor!

    } catch (Exception& e) {
      throw fawkes::Exception("Could not add robot '%s' to openrave environment. (Error: %s)", __cfg_OR_robot_file.c_str(), e.what_no_backtrace());
    }
  }

  while( !__robot ) {
    __robot = __viewer_env.robot->get_robot_ptr();
    usleep(100);
  }
  while( !__manip ) {
    __manip = __robot->SetActiveManipulator(__manipname);
    usleep(100);
  }

  // create cloned environment for planning
  logger->log_debug(name(), "Clone environment for planning");
  openrave->clone(&__planner_env.env, &__planner_env.robot, &__planner_env.manip);

  if( __planner_env.env == NULL
   || __planner_env.robot == NULL
   || __planner_env.manip == NULL) {
    throw fawkes::Exception("Could not clone properly, received a NULL pointer");
  }

  // set active manipulator in planning environment. This won't change anymore!
  RobotBase::ManipulatorPtr manip = __planner_env.robot->get_robot_ptr()->SetActiveManipulator(__manipname);
  __planner_env.robot->get_robot_ptr()->SetActiveDOFs(manip->GetArmIndices());

#endif //HAVE_OPENRAVE
}

void
KinovaOpenraveSingleThread::finalize() {
#ifdef HAVE_OPENRAVE
  delete(__planner_env.robot);
  __planner_env.robot = NULL;

  delete(__planner_env.manip);
  __planner_env.manip = NULL;

  delete(__planner_env.env);
  __planner_env.env = NULL;

  if(!__load_robot) {
    // avoid implicit deletes or anything the like
    __viewer_env.robot = NULL;
    __viewer_env.manip = NULL;
    __viewer_env.env = NULL;
  } else {
    KinovaOpenraveBaseThread::finalize();
  }
#endif
}

void
KinovaOpenraveSingleThread::loop()
{
  if( __arm == NULL ) {
    usleep(30e3);
    return;
  }

  __planning_mutex->lock();
  RefPtr<jaco_target_t> from, to;
  // get first target with type TARGET_TRAJEC that needs a planner
  __target_mutex->lock();
  for( jaco_target_queue_t::iterator it=__target_queue->begin(); it!=__target_queue->end(); ++it ) {
    if( (*it)->type == TARGET_TRAJEC && !(*it)->trajec ) {
      // have found a new target for path planning!
      to = *it;

      // check if there is a prior target that needs to be considered in planning
      if( it!=__target_queue->begin() ) {
        --it;
        from = *it;
        ++it;
      }

      break;
    }
  }
  __target_mutex->unlock();

  if( to ) {
    if( !from )
      from = RefPtr<jaco_target_t>(new jaco_target_t());

    if( from->pos.size() != 6 ) {
      __arm->arm->get_joints(from->pos);
    }

    // run planner
    _plan_path(from, to);
    __planning_mutex->unlock();

  } else {
    __planning_mutex->unlock();
    usleep(30e3); // TODO: make this configurable
  }
}

void
KinovaOpenraveSingleThread::register_arm(jaco_arm_t *arm)
{
  __arm = arm;
  __target_mutex = __arm->target_mutex;
  __trajec_mutex = __arm->trajec_mutex;
  __target_queue = __arm->target_queue;
}

void
KinovaOpenraveSingleThread::unregister_arms() {
  __arm = NULL;
}

void
KinovaOpenraveSingleThread::update_openrave()
{
  if( __arm == NULL || __robot == NULL || __manip == NULL )
    return;

#ifdef HAVE_OPENRAVE
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

    __robot->SetDOFValues(__joints, 1, __manip->GetArmIndices());

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
#endif
}

bool
KinovaOpenraveSingleThread::add_target(float x, float y, float z, float e1, float e2, float e3, bool plan)
{
  bool solvable = false;

#ifdef HAVE_OPENRAVE
  try {
    // update planner params; set correct DOF and stuff
    __planner_env.robot->get_planner_params()->SetRobotActiveJoints(__planner_env.robot->get_robot_ptr());
    __planner_env.robot->get_planner_params()->vgoalconfig.resize(__planner_env.robot->get_robot_ptr()->GetActiveDOF());

    // get IK from openrave
    solvable = __planner_env.robot->set_target_euler(EULER_ZXZ, x, y, z, e1, e2, e3);

    if( solvable ) {
      logger->log_debug(name(), "IK successful!");

      // create new target for the queue
      RefPtr<jaco_target_t> target(new jaco_target_t());

      // get target IK valoues
      __planner_env.robot->get_target().manip->get_angles_device(target->pos);

      if( plan ) {
        // add this to the target queue for planning
        logger->log_debug(name(), "Adding to target_queue for later planning");
        target->type = TARGET_TRAJEC;

      } else {
        // don't plan, consider this the final configuration
        logger->log_debug(name(), "Skip planning, add this as TARGET_ANGULAR");
        target->type = TARGET_ANGULAR;
      }
      __target_mutex->lock();
      __target_queue->push_back(target);
      __target_mutex->unlock();

    } else {
      logger->log_warn(name(), "No IK solution found for target.");
      return solvable;
    }

  } catch( openrave_exception &e) {
    throw fawkes::Exception("OpenRAVE Exception:%s", e.what());
  }
#endif

  return solvable;
}

bool
KinovaOpenraveSingleThread::set_target(float x, float y, float z, float e1, float e2, float e3, bool plan)
{
  __target_mutex->lock();
  __target_queue->clear();
  __target_mutex->unlock();
  return add_target(x, y, z, e1, e2, e3, plan);
}

void
KinovaOpenraveSingleThread::_plan_path(RefPtr<jaco_target_t> &from, RefPtr<jaco_target_t> &to)
{
  // Set target point for planner (has already passed IK check previously!)
  __planner_env.manip->set_angles_device(to->pos);
  std::vector<float> target;
  __planner_env.manip->get_angles(target);

  //logger->log_debug(name(), "setting target %f %f %f %f %f %f",
  //                  to->pos.at(0), to->pos.at(1), to->pos.at(2), to->pos.at(3), to->pos.at(4), to->pos.at(5));
  __planner_env.robot->set_target_angles(target);

  // Set starting point for planner, convert encoder values to angles if necessary
  //logger->log_debug(name(), "setting start %f %f %f %f %f %f",
  //                  from->pos.at(0), from->pos.at(1), from->pos.at(2), from->pos.at(3), from->pos.at(4), from->pos.at(5));
  __planner_env.manip->set_angles_device(from->pos);

  // Set planning parameters (none yet)
  __planner_env.robot->set_target_plannerparams("");

  // Run planner
  float sampling = 0.01f; //maybe catch from config? or "learning" depending on performance?
  try {
    __planner_env.env->run_planner(__planner_env.robot, sampling);
  } catch (fawkes::Exception &e) {
    logger->log_warn(name(), "Planning failed: %s", e.what_no_backtrace());
    // TODO: better handling!
    // for now just skip planning, so the target_queue can be processed
    __target_mutex->lock();
    to->type = TARGET_ANGULAR;
    __target_mutex->unlock();
    return;
  }

  // add trajectory to queue
  //logger->log_debug(name(), "plan successful, adding to queue");
  __trajec_mutex->lock();
  // we can do the following becaouse get_trajectory_device() returns a new object, thus
  //  can be safely deleted by RefPtr auto-deletion
  to->trajec = RefPtr<jaco_trajec_t>( __planner_env.robot->get_trajectory_device() );
  __trajec_mutex->unlock();
}



/** Plot the first target of the queue in the viewer_env */
void
KinovaOpenraveSingleThread::plot_first()
{
#ifdef HAVE_OPENRAVE
  if( !__cfg_OR_use_viewer )
    return;

  // check if there is a target to be plotted
  __target_mutex->lock();
  if( __target_queue->empty() ) {
    __target_mutex->unlock();
    return;
  }

  // get RefPtr to first target in queue
  RefPtr<jaco_target_t> target = __target_queue->front();
  __target_mutex->unlock();


  // only plot trajectories
  if( target->type != TARGET_TRAJEC )
    return;

  // plot the trajectory (if possible)
  __trajec_mutex->lock();
  if( !target->trajec ) {
    __trajec_mutex->unlock();
    return;
  }

  // remove all GraphHandlerPtr and currently drawn plots
  __graph_handle.clear();
  {
    // save the state, do not modifiy currently active robot!
    RobotBasePtr tmp_robot = __viewer_env.robot->get_robot_ptr();
    RobotBase::RobotStateSaver saver(tmp_robot);

    OpenRAVE::Vector color(__arm->trajec_color[0],
                           __arm->trajec_color[1],
                           __arm->trajec_color[2],
                           __arm->trajec_color[3]);
    std::vector<dReal> joints;
    OpenRaveManipulator* manip = __viewer_env.manip->copy();

    for(jaco_trajec_t::iterator it = target->trajec->begin(); it!=target->trajec->end(); ++it) {
      manip->set_angles_device((*it));
      manip->get_angles(joints);

      tmp_robot->SetDOFValues(joints, 1, __manip->GetArmIndices());

      const OpenRAVE::Vector &trans = __manip->GetEndEffectorTransform().trans;
      float transa[4] = { (float)trans.x, (float)trans.y, (float)trans.z, (float)trans.w };
      __graph_handle.push_back( __viewer_env.env->get_env_ptr()->plot3(transa,1, 0, 2.f, color));
    }
  } // robot state is restored

  __trajec_mutex->unlock();

#endif //HAVE_OPENRAVE
}
