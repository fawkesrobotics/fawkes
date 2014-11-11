
/***************************************************************************
 *  environment.cpp - Fawkes to OpenRAVE Environment
 *
 *  Created: Sun Sep 19 14:50:34 2010
 *  Copyright  2010  Bahram Maleki-Fard, AllemaniACs RoboCup Team
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

// must be first because it redefines macros from standard headers
#include <Python.h>

#include "environment.h"
#include "robot.h"
#include "manipulator.h"

#include <logging/logger.h>
#include <core/exceptions/software.h>

#include <openrave-core.h>
#include <openrave/planningutils.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <sstream>
#include <cstdio>

using namespace OpenRAVE;
namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif


/** Sets and loads a viewer for OpenRAVE.
 * @param env OpenRAVE environment to be attached
 * @param viewername name of the viewr, usually "qtcoin"
 * @param running pointer to a local variable, which will be set to "true"
 *  as long as the viewer thread runs, and "false" when the GUI closes.
 */
void
run_viewer(OpenRAVE::EnvironmentBasePtr env, const std::string& viewername, bool* running)
{
  ViewerBasePtr viewer = RaveCreateViewer(env, viewername);
  env->Add(viewer);
  //call the viewer's infinite loop (this is why you need a separate thread):
  *running = true;
  viewer->main(/*showGUI=*/true);
  *running = false;
  env->Remove(viewer);
}


/** @class OpenRaveEnvironment <plugins/openrave/environment.h>
* Class handling interaction with the OpenRAVE::EnvironmentBase class.
* This class loads a scene and handles robots/objects etc in it. All calculations
* in OpenRAVE (IK, planning, etc) are done based on the current scene.
* @author Bahram Maleki-Fard
*/

/** Constructor.
 * @param logger pointer to fawkes logger
 */
OpenRaveEnvironment::OpenRaveEnvironment(fawkes::Logger* logger) :
  __logger( logger ),
  __viewer_thread( 0 ),
  __viewer_running( 0 )
{
  set_name("");
}

/** Copy constructor.
 * This also clones the environment in OpenRAVE, including all bodies!
 * BiRRT planner and IKFast module are also created.
 * @param src The OpenRaveEnvironment to clone
 */
OpenRaveEnvironment::OpenRaveEnvironment(const OpenRaveEnvironment& src)
 : __logger( src.__logger ),
   __viewer_thread( 0 ),
   __viewer_running( 0 )
{
  __env = src.__env->CloneSelf(OpenRAVE::Clone_Bodies);

  // update name string
  set_name( src.__name.c_str() );

  // create planner
  __planner = RaveCreatePlanner(__env,"birrt");
  if(!__planner)
    throw fawkes::Exception("%s: Could not create planner. Error in OpenRAVE.", name());

  // create ikfast module
  __mod_ikfast = RaveCreateModule(__env,"ikfast");
  __env->AddModule(__mod_ikfast,"");

  if (__logger)
    __logger->log_debug(name(), "Environment cloned from %s", src.__name.c_str());
}

/** Destructor. */
OpenRaveEnvironment::~OpenRaveEnvironment()
{
  this->destroy();
}

/** Create and lock the environment. */
void
OpenRaveEnvironment::create()
{
  // create environment
  __env = RaveCreateEnvironment();
  if(!__env) {
    throw fawkes::Exception("%s: Could not create environment. Error in OpenRAVE.", name());
  } else {
    // update name_string
    set_name(__name.c_str());
    if (__logger)
      __logger->log_debug(name(), "Environment created");
  }

  EnvironmentMutex::scoped_lock( __env->GetMutex());

  // create planner
  __planner = RaveCreatePlanner(__env,"birrt");
  if(!__planner)
    throw fawkes::Exception("%s: Could not create planner. Error in OpenRAVE.", name());

  // create ikfast module
  __mod_ikfast = RaveCreateModule(__env,"ikfast");
  __env->AddModule(__mod_ikfast,"");
}

/** Destroy the environment. */
void
OpenRaveEnvironment::destroy()
{
  if( __viewer_thread ) {
    __viewer_thread->detach();
    __viewer_thread->join();
    delete __viewer_thread;
    __viewer_thread=NULL;
  }

  try {
    __env->Destroy();
    if(__logger)
      __logger->log_debug(name(), "Environment destroyed");
  } catch(const openrave_exception& e) {
    if(__logger)
      __logger->log_warn(name(), "Could not destroy Environment. Ex:%s", e.what());
  }
}

/** Get the name string to use in logging messages etc. */
const char*
OpenRaveEnvironment::name() const
{
  return __name_str.c_str();
}

/** Set name of environment.
 * Nothing important, but helpful for debugging etc.
 * @param name The name of the environment. Can be an empty string.
 */
void
OpenRaveEnvironment::set_name(const char* name)
{
  std::stringstream n;
  n << "OpenRaveEnvironment" << "[";
  if( __env )
    n << RaveGetEnvironmentId(__env) << ":";
  n << name << "]";
  __name_str = n.str();

  if (__logger)
    __logger->log_debug(__name_str.c_str(), "Set environment name (previously '%s')", __name.c_str());
  __name = name;
}

/** Enable debugging messages of OpenRAVE.
 * @param level debug level to set for OpenRAVE
 */
void
OpenRaveEnvironment::enable_debug(OpenRAVE::DebugLevel level)
{
  RaveSetDebugLevel(level);
}

/** Disable debugging messages of OpenRAVE. */
void
OpenRaveEnvironment::disable_debug()
{
  RaveSetDebugLevel(Level_Fatal);
}

/** Add a robot into the scene.
 * @param robot RobotBasePtr of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
void
OpenRaveEnvironment::add_robot(OpenRAVE::RobotBasePtr robot)
{
  try{
    EnvironmentMutex::scoped_lock( __env->GetMutex());
    __env->Add(robot);
    if(__logger)
      __logger->log_debug(name(), "Robot '%s' added to environment.", robot->GetName().c_str());
  } catch(openrave_exception &e) {
    if(__logger)
      __logger->log_debug(name(), "Could not add robot '%s' to environment. OpenRAVE error:%s", robot->GetName().c_str(), e.message().c_str());
  }
}

/** Add a robot into the scene.
 * @param filename path to robot's xml file
 * @return 1 if succeeded, 0 if not able to load file
 */
void
OpenRaveEnvironment::add_robot(const std::string& filename)
{
  RobotBasePtr robot;
  {
    // load the robot
    EnvironmentMutex::scoped_lock( __env->GetMutex());
    robot = __env->ReadRobotXMLFile(filename);
  }

  // if could not load robot file: Check file path, and test file itself for correct syntax and semantics
  // by loading it directly into openrave with "openrave robotfile.xml"
  if( !robot )
    throw fawkes::IllegalArgumentException("%s: Robot '%s' could not be loaded. Check xml file/path.", name(), filename.c_str());
  else if(__logger)
    __logger->log_debug(name(), "Robot '%s' loaded.", robot->GetName().c_str());

  add_robot(robot);
}

/** Add a robot into the scene.
 * @param robot pointer to OpenRaveRobot object of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
void
OpenRaveEnvironment::add_robot(OpenRaveRobotPtr& robot)
{
  add_robot(robot->get_robot_ptr());
}


/** Get EnvironmentBasePtr.
 * @return EnvironmentBasePtr in use
 */
OpenRAVE::EnvironmentBasePtr
OpenRaveEnvironment::get_env_ptr() const
{
  return __env;
}

/** Starts the  qt viewer in a separate thread.
 *  Use this mainly for debugging purposes, as it uses
 *  a lot of CPU/GPU resources.
 */
void
OpenRaveEnvironment::start_viewer()
{
  if( __viewer_running )
    return;

  if( __viewer_thread ) {
    __viewer_thread->join();
    delete __viewer_thread;
    __viewer_thread = NULL;
  }

  try {
    // set this variable to true here already. Otherwise we would have to wait for the upcoming
    // boost thread to start, create viewer and add viewer to environment to get this variable set
    // to "true". Another call to "start_viewer()" would get stuck then, waiting for "join()"!
    __viewer_running = true;
    __viewer_thread = new boost::thread(boost::bind(run_viewer, __env, "qtcoin", &__viewer_running));
  } catch( const openrave_exception &e) {
    __viewer_running = false;
    if(__logger)
      __logger->log_error(name(), "Could not load viewr. Ex:%s", e.what());
    throw;
  }
}


/** Autogenerate IKfast IK solver for robot.
 * @param robot pointer to OpenRaveRobot object
 * @param iktype IK type of solver (default: Transform6D; use TranslationDirection5D for 5DOF arms)
 */
void
OpenRaveEnvironment::load_IK_solver(OpenRaveRobotPtr& robot, OpenRAVE::IkParameterizationType iktype)
{
  EnvironmentMutex::scoped_lock( __env->GetMutex());

  RobotBasePtr robotBase = robot->get_robot_ptr();

  std::stringstream ssin,ssout;
  ssin << "LoadIKFastSolver " << robotBase->GetName() << " " << (int)iktype;
  // if necessary, add free inc for degrees of freedom
  //ssin << " " << 0.04f;
  if( !__mod_ikfast->SendCommand(ssout,ssin) )
    throw fawkes::Exception("%s: Could not load ik solver", name());
}

/** Plan collision-free path for current and target manipulator
 * configuration of a OpenRaveRobot robot.
 * @param robot pointer to OpenRaveRobot object of robot to use
 * @param sampling sampling time between each trajectory point (in seconds)
 */
void
OpenRaveEnvironment::run_planner(OpenRaveRobotPtr& robot, float sampling)
{
  bool success;
  EnvironmentMutex::scoped_lock lock(__env->GetMutex()); // lock environment

  robot->get_planner_params(); // also updates internal __manip

  /*
  // init planner. This is automatically done by BaseManipulation, but putting it here
  // helps to identify problem source if any occurs.
  success = __planner->InitPlan(robot->get_robot_ptr(),robot->get_planner_params());
  if(!success)
    {throw fawkes::Exception("%s: Planner: init failed", name());}
  else if(__logger)
    {__logger->log_debug(name(), "Planner: initialized");}
  */
  // plan path with basemanipulator
  ModuleBasePtr basemanip = robot->get_basemanip();
  target_t target = robot->get_target();
  std::stringstream cmdin,cmdout;
  cmdin << std::setprecision(std::numeric_limits<dReal>::digits10+1);
  cmdout << std::setprecision(std::numeric_limits<dReal>::digits10+1);

  if( target.type == TARGET_RELATIVE_EXT ) {
    Transform t = robot->get_robot_ptr()->GetActiveManipulator()->GetEndEffectorTransform();
    //initial pose of arm looks at +z. Target values are referring to robot's coordinating system,
    //which have this direction vector if taken as extension of manipulator (rotate -90° on y-axis)
    Vector dir(target.y,target.z,target.x);
    TransformMatrix mat = matrixFromQuat(t.rot);
    dir = mat.rotate(dir);
    target.type = TARGET_RELATIVE;
    target.x = dir[0];
    target.y = dir[1];
    target.z = dir[2];
  }

  switch(target.type) {
    case (TARGET_RAW) :
      cmdin << target.raw_cmd;
      break;

    case (TARGET_JOINTS) :
      cmdin << "MoveActiveJoints goal";
      {
        std::vector<dReal> v;
        target.manip->get_angles(v);
        for(size_t i = 0; i < v.size(); ++i) {
          cmdin << " " << v[i];
        }
      }
      break;

    case (TARGET_IKPARAM) :
      cmdin << "MoveToHandPosition ikparam";
      cmdin << " " << target.ikparam;
      break;

    case (TARGET_TRANSFORM) :
      cmdin << "MoveToHandPosition pose";
      cmdin << " " << target.qw << " " << target.qx << " " << target.qy << " " << target.qz;
      cmdin << " " << target.x  << " " << target.y  << " " << target.z;
      break;

    case (TARGET_RELATIVE) :
      // for now move to all relative targets in a straigt line
      cmdin << "MoveHandStraight direction";
      cmdin << " " << target.x << " " << target.y << " " << target.z;
      {
        dReal stepsize = 0.005;
        dReal length = sqrt(target.x*target.x + target.y*target.y + target.z*target.z);
        int minsteps = (int)(length/stepsize);
        if( minsteps > 4 )
          minsteps -= 4;

        cmdin << " stepsize " << stepsize;
        cmdin << " minsteps " << minsteps;
        cmdin << " maxsteps " << (int)(length/stepsize);
      }
      break;

    default :
      throw fawkes::Exception("%s: Planner: Invalid target type", name());
  }

  //add additional planner parameters
  if( !target.plannerparams.empty() ) {
    cmdin << " " << target.plannerparams;
  }
  cmdin << " execute 0";
  cmdin << " outputtraj";
  if(__logger)
    __logger->log_debug(name(), "Planner: basemanip cmdin:%s", cmdin.str().c_str());

  try {
    success = basemanip->SendCommand(cmdout,cmdin);
  } catch(openrave_exception &e) {
    throw fawkes::Exception("%s: Planner: basemanip command failed. Ex%s", name(), e.what());
  }
  if(!success)
    throw fawkes::Exception("%s: Planner: planning failed", name());
  else if(__logger)
    __logger->log_debug(name(), "Planner: path planned");

  if(__logger)
    __logger->log_debug(name(), "Planner: planned. cmdout:%s", cmdout.str().c_str());

  // read returned trajectory
  TrajectoryBasePtr traj = RaveCreateTrajectory(__env, "");
  traj->Init(robot->get_robot_ptr()->GetActiveConfigurationSpecification());
  if( !traj->deserialize(cmdout) )
    throw fawkes::Exception("%s: Planner: Cannot read trajectory data.", name());

  // sampling trajectory and setting robots trajectory
  std::vector< std::vector<dReal> >* trajRobot = robot->get_trajectory();
  trajRobot->clear();
  for(dReal time = 0; time <= traj->GetDuration(); time += (dReal)sampling) {
    std::vector<dReal> point;
    traj->Sample(point,time);
    trajRobot->push_back(point);
  }

  // viewer options
  if( __viewer_running ) {

    // display trajectory in viewer
    __graph_handle.clear(); // remove all GraphHandlerPtr and currently drawn plots
    {
      RobotBasePtr tmp_robot = robot->get_robot_ptr();
      RobotBase::RobotStateSaver saver(tmp_robot); // save the state, do not modifiy currently active robot!
        for(std::vector< std::vector<dReal> >::iterator it = trajRobot->begin(); it!=trajRobot->end(); ++it) {
          tmp_robot->SetActiveDOFValues((*it));
          const OpenRAVE::Vector &trans = tmp_robot->GetActiveManipulator()->GetEndEffectorTransform().trans;
          float transa[4] = { (float)trans.x, (float)trans.y, (float)trans.z, (float)trans.w };
          __graph_handle.push_back(__env->plot3(transa, 1, 0, 2.f, Vector(1.f, 0.f, 0.f, 1.f)));
        }
     } // robot state is restored

    // display motion in viewer
    if( robot->display_planned_movements()) {
      robot->get_robot_ptr()->GetController()->SetPath(traj);
    }
  }

}


/** Run graspplanning script for a given target.
 * Script loads grasping databse, checks if target is graspable for various grasps
 * and on success returns a string containing trajectory data.
 * Currently the grasping databse can only be accessed via python, so we use a short
 * python script (shortened and modified from officiel OpenRAVE graspplanning.py) to do the work.
 * @param target_name name of targeted object (KinBody)
 * @param robot pointer to OpenRaveRobot object of robot to use
 * @param sampling sampling time between each trajectory point (in seconds)
 */
void
OpenRaveEnvironment::run_graspplanning(const std::string& target_name, OpenRaveRobotPtr& robot, float sampling)
{
  std::string filename = SRCDIR"/python/graspplanning.py";
  std::string funcname = "runGrasp";

  TrajectoryBasePtr traj = RaveCreateTrajectory(__env, "");
  traj->Init(robot->get_robot_ptr()->GetActiveConfigurationSpecification());

  FILE* py_file = fopen(filename.c_str(), "r");
  if (py_file == NULL)
    throw fawkes::Exception("%s: Graspplanning: opening python file failed", name());

  Py_Initialize();

  // Need to aquire global interpreter lock (GIL), create new sub-interpreter to run code in there
  PyGILState_STATE gil_state = PyGILState_Ensure(); // aquire python GIL
  PyThreadState* cur_state = PyThreadState_Get();   // get current ThreadState; need this to switch back to later
  PyThreadState* int_state = Py_NewInterpreter();   // create new sub-interpreter
  PyThreadState_Swap(int_state);                    // set active ThreadState; maybe not needed after calling NewInterpreter() ?
  // Now we can safely run our python code

  // using python C API
  PyObject* py_main = PyImport_AddModule("__main__"); // borrowed reference
  if( !py_main ) {
    // __main__ should always exist
    fclose(py_file);
    Py_EndInterpreter(int_state);
    PyThreadState_Swap(cur_state);
    PyGILState_Release(gil_state); // release GIL
    Py_Finalize();
    throw fawkes::Exception("%s: Graspplanning: Python reference '__main__' does not exist.", name());
  }
  PyObject* py_dict = PyModule_GetDict(py_main);      // borrowed reference
  if( !py_dict ) {
    // __main__ should have a dictionary
    fclose(py_file);
    Py_Finalize();
    throw fawkes::Exception("%s: Graspplanning: Python reference '__main__' does not have a dictionary.", name());
  }

  // load file
  int py_module = PyRun_SimpleFile(py_file, filename.c_str());
  fclose(py_file);
  if (!py_module) {
    // load function from global dictionary
    PyObject* py_func = PyDict_GetItemString(py_dict, funcname.c_str());
    if (py_func && PyCallable_Check(py_func)) {
      // create tuple for args to be passed to py_func
      PyObject* py_args = PyTuple_New(3);
      // fill tuple with values. We're not checking for conversion errors here atm, can be added!
      PyObject* py_value_env_id      = PyInt_FromLong(RaveGetEnvironmentId(__env));
      PyObject* py_value_robot_name  = PyString_FromString(robot->get_robot_ptr()->GetName().c_str());
      PyObject* py_value_target_name = PyString_FromString(target_name.c_str());
      PyTuple_SetItem(py_args, 0, py_value_env_id);      //py_value reference stolen here! no need to DECREF
      PyTuple_SetItem(py_args, 1, py_value_robot_name);  //py_value reference stolen here! no need to DECREF
      PyTuple_SetItem(py_args, 2, py_value_target_name); //py_value reference stolen here! no need to DECREF
      // call function, get return value
      PyObject* py_value = PyObject_CallObject(py_func, py_args);
      Py_DECREF(py_args);
      // check return value
      if (py_value != NULL) {
        if (!PyString_Check(py_value)) {
          Py_DECREF(py_value);
          Py_DECREF(py_func);
          Py_Finalize();
          throw fawkes::Exception("%s: Graspplanning: No grasping path found.", name());
        }
        std::stringstream resval;
        resval << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        resval << PyString_AsString(py_value);
        if (!traj->deserialize(resval) ) {
          Py_DECREF(py_value);
          Py_DECREF(py_func);
          Py_Finalize();
          throw fawkes::Exception("%s: Graspplanning: Reading trajectory data failed.", name());
        }
        Py_DECREF(py_value);
      } else { // if calling function failed
        Py_DECREF(py_func);
        PyErr_Print();
        Py_Finalize();
        throw fawkes::Exception("%s: Graspplanning: Calling function failed.", name());
      }
    } else { // if loading func failed
      if (PyErr_Occurred())
        PyErr_Print();
      Py_XDECREF(py_func);
      Py_Finalize();
      throw fawkes::Exception("%s: Graspplanning: Loading function failed.", name());
    }
    Py_XDECREF(py_func);
  } else { // if loading module failed
    PyErr_Print();
    Py_Finalize();
    throw fawkes::Exception("%s: Graspplanning: Loading python file failed.", name());
  }

  Py_EndInterpreter(int_state); // close sub-interpreter
  PyThreadState_Swap(cur_state); // re-set active state to previous one
  PyGILState_Release(gil_state); // release GIL

  Py_Finalize(); // should be careful with that, as it closes global interpreter; Other threads running python may fail

  if(__logger)
    __logger->log_debug(name(), "Graspplanning: path planned");

  // re-timing the trajectory with
  planningutils::RetimeActiveDOFTrajectory(traj, robot->get_robot_ptr());

  // sampling trajectory and setting robots trajectory
  std::vector< std::vector<dReal> >* trajRobot = robot->get_trajectory();
  trajRobot->clear();
  for(dReal time = 0; time <= traj->GetDuration(); time += (dReal)sampling) {
    std::vector<dReal> point;
    traj->Sample(point,time);
    trajRobot->push_back(point);
  }

  // viewer options
  if( __viewer_running ) {

    // display trajectory in viewer
    __graph_handle.clear(); // remove all GraphHandlerPtr and currently drawn plots
    {
      RobotBasePtr tmp_robot = robot->get_robot_ptr();
      RobotBase::RobotStateSaver saver(tmp_robot); // save the state, do not modifiy currently active robot!
        for(std::vector< std::vector<dReal> >::iterator it = trajRobot->begin(); it!=trajRobot->end(); ++it) {
          tmp_robot->SetActiveDOFValues((*it));
          const OpenRAVE::Vector &trans = tmp_robot->GetActiveManipulator()->GetEndEffectorTransform().trans;
          float transa[4] = { (float)trans.x, (float)trans.y, (float)trans.z, (float)trans.w };
          __graph_handle.push_back(__env->plot3(transa, 1, 0, 2.f, Vector(1.f, 0.f, 0.f, 1.f)));
        }
     } // robot state is restored

    // display motion in viewer
    if( robot->display_planned_movements()) {
      robot->get_robot_ptr()->GetController()->SetPath(traj);
    }
  }
}


/** Add an object to the environment.
 * @param name name that should be given to that object
 * @param filename path to xml file of that object (KinBody)
 * @return true if successful
 */
bool
OpenRaveEnvironment::add_object(const std::string& name, const std::string& filename)
{
  try {
    EnvironmentMutex::scoped_lock lock(__env->GetMutex());
    KinBodyPtr kb = __env->ReadKinBodyXMLFile(filename);
    kb->SetName(name);
    __env->Add(kb);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(this->name(), "Could not add Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Remove object from environment.
 * @param name name of the object
 * @return true if successful
 */
bool
OpenRaveEnvironment::delete_object(const std::string& name)
{
  try {
    EnvironmentMutex::scoped_lock lock(__env->GetMutex());
    KinBodyPtr kb = __env->GetKinBody(name);
    __env->Remove(kb);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(this->name(), "Could not delete Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Remove all objects from environment.
 * @return true if successful
 */
bool
OpenRaveEnvironment::delete_all_objects()
{
  try {
    EnvironmentMutex::scoped_lock lock(__env->GetMutex());
    std::vector<KinBodyPtr> bodies;
    __env->GetBodies( bodies );

    for( std::vector<KinBodyPtr>::iterator it=bodies.begin(); it!=bodies.end(); ++it ) {
      if( !(*it)->IsRobot() )
        __env->Remove(*it);
    }
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(this->name(), "Could not delete all objects. Ex:%s", e.what());
    return false;
  }

  return true;
}

/** Rename object.
 * @param name current name of the object
 * @param new_name new name of the object
 * @return true if successful
 */
bool
OpenRaveEnvironment::rename_object(const std::string& name, const std::string& new_name)
{
  try {
    EnvironmentMutex::scoped_lock lock(__env->GetMutex());
    KinBodyPtr kb = __env->GetKinBody(name);
    kb->SetName(new_name);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(this->name(), "Could not rename Object '%s' to '%s'. Ex:%s", name.c_str(), new_name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Move object in the environment.
 * Distances are given in meters
 * @param name name of the object
 * @param trans_x transition along x-axis
 * @param trans_y transition along y-axis
 * @param trans_z transition along z-axis
 * @return true if successful
 */
bool
OpenRaveEnvironment::move_object(const std::string& name, float trans_x, float trans_y, float trans_z)
{
  try {
    EnvironmentMutex::scoped_lock lock(__env->GetMutex());
    KinBodyPtr kb = __env->GetKinBody(name);

    Transform transform = kb->GetTransform();
    transform.trans = Vector(trans_x, trans_y, trans_z);

    kb->SetTransform(transform);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(this->name(), "Could not move Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Move object in the environment.
 * Distances are given in meters
 * @param name name of the object
 * @param trans_x transition along x-axis
 * @param trans_y transition along y-axis
 * @param trans_z transition along z-axis
 * @param robot move relatively to robot (in most simple cases robot is at position (0,0,0) anyway, so this has no effect)
 * @return true if successful
 */
bool
OpenRaveEnvironment::move_object(const std::string& name, float trans_x, float trans_y, float trans_z, OpenRaveRobotPtr& robot)
{
  // remember, OpenRAVE Vector is 4-tuple (w,x,y,z)
  Transform t;
  {
    EnvironmentMutex::scoped_lock( __env->GetMutex());
    t = robot->get_robot_ptr()->GetTransform();
  }
  return move_object(name, trans_x+t.trans[1], trans_y+t.trans[2], trans_z+t.trans[3]);
}

/** Rotate object by a quaternion.
 * @param name name of the object
 * @param quat_x x value of quaternion
 * @param quat_y y value of quaternion
 * @param quat_z z value of quaternion
 * @param quat_w w value of quaternion
 * @return true if successful
 */
bool
OpenRaveEnvironment::rotate_object(const std::string& name, float quat_x, float quat_y, float quat_z, float quat_w)
{
  try {
    EnvironmentMutex::scoped_lock lock(__env->GetMutex());
    KinBodyPtr kb = __env->GetKinBody(name);

    Vector quat(quat_w, quat_x, quat_y, quat_z);

    Transform transform = kb->GetTransform();
    transform.rot = quat;

    kb->SetTransform(transform);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn(this->name(), "Could not rotate Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

/** Rotate object along its axis.
 * Rotation angles should be given in radians.
 * @param name name of the object
 * @param rot_x 1st rotation, along x-axis
 * @param rot_y 2nd rotation, along y-axis
 * @param rot_z 3rd rotation, along z-axis
 * @return true if successful
 */
bool
OpenRaveEnvironment::rotate_object(const std::string& name, float rot_x, float rot_y, float rot_z)
{
  Vector q1 = quatFromAxisAngle(Vector(rot_x, 0.f, 0.f));
  Vector q2 = quatFromAxisAngle(Vector(0.f, rot_y, 0.f));
  Vector q3 = quatFromAxisAngle(Vector(0.f, 0.f, rot_z));

  Vector q12  = quatMultiply (q1, q2);
  Vector quat = quatMultiply (q12, q3);

  return rotate_object(name, quat[1], quat[2], quat[3], quat[0]);
}


/** Clone all non-robot objects from a referenced OpenRaveEnvironment to this one.
 * The environments should contain the same objects afterwards. Therefore objects in current
 *  environment that do not exist in the reference environment are deleted as well.
 * @param env The reference environment
 */
void
OpenRaveEnvironment::clone_objects(OpenRaveEnvironmentPtr& env)
{
  // lock environments
  EnvironmentMutex::scoped_lock lockold(env->get_env_ptr()->GetMutex());
  EnvironmentMutex::scoped_lock lock(__env->GetMutex());

  // get kinbodies
  std::vector<KinBodyPtr> old_bodies, bodies;
  env->get_env_ptr()->GetBodies( old_bodies );
  __env->GetBodies( bodies );

  // check for existing bodies in this environment
  std::vector<KinBodyPtr>::iterator old_body, body;
  for(old_body=old_bodies.begin(); old_body!=old_bodies.end(); ++old_body ) {
    if( (*old_body)->IsRobot() )
      continue;

    KinBodyPtr new_body;
    for( body=bodies.begin(); body!=bodies.end(); ++body ) {
      if( (*body)->IsRobot() )
        continue;

      if( (*body)->GetName() == (*old_body)->GetName() && (*body)->GetKinematicsGeometryHash() == (*old_body)->GetKinematicsGeometryHash() ) {
        new_body = *body;
        break;
      }
    }

    if( body != bodies.end() ) {
      // remove this one from the list, to reduce checking
      // (this one has already been found a match)
      bodies.erase( body );
    }

    if( !new_body ) {
      // this is a new kinbody!

      // create new empty KinBody, then clone from old
      KinBodyPtr empty;
      new_body = __env->ReadKinBodyData(empty, "<KinBody></KinBody>");
      new_body->Clone(*old_body, 0);

      // add kinbody to environment
      __env->Add(new_body);

      // update collisison-checker and physics-engine to consider new kinbody
      //__env->GetCollisionChecker()->InitKinBody(new_body);
      //__env->GetPhysicsEngine()->InitKinBody(new_body);

      // clone kinbody state
      KinBody::KinBodyStateSaver saver(*old_body, KinBody::Save_LinkTransformation|KinBody::Save_LinkEnable|KinBody::Save_LinkVelocities);
      saver.Restore(new_body);

    } else {
      // this kinbody already exists. just clone the state
      KinBody::KinBodyStateSaver saver(*old_body, 0xffffffff);
      saver.Restore(new_body);
    }
  }

  // remove bodies that are not in old_env anymore
  for( body=bodies.begin(); body!=bodies.end(); ++body ) {
    if( (*body)->IsRobot() )
      continue;

    __env->Remove( *body );
  }
}

} // end of namespace fawkes
