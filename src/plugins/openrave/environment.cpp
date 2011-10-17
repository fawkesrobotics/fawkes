
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
#include "environment.h"
#include "robot.h"

#include <openrave-core.h>
#include <utils/logging/logger.h>
#include <core/exceptions/software.h>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <Python.h>

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
 */
void
SetViewer(OpenRAVE::EnvironmentBasePtr env, const std::string& viewername)
{
  ViewerBasePtr viewer = RaveCreateViewer(env, viewername);

  // attach it to the environment:
  env->AddViewer(viewer);

  // finally you call the viewer's infinite loop (this is why you need a separate thread):
  viewer->main(/*showGUI=*/true);
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
  __viewer_enabled( 0 )
{
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
  if(!__env)
    {throw fawkes::Exception("OpenRAVE Environment: Could not create environment. Error in OpenRAVE.");}
  else if (__logger)
    {__logger->log_debug("OpenRAVE Environment", "Environment created");}

  // create planner
  __planner = RaveCreatePlanner(__env,"birrt");
  if(!__planner)
    {throw fawkes::Exception("OpenRAVE Environment: Could not create planner. Error in OpenRAVE.");}

  // create ikfast module
  __mod_ikfast = RaveCreateModule(__env,"ikfast");
  __env->AddModule(__mod_ikfast,"");
}

/** Destroy the environment. */
void
OpenRaveEnvironment::destroy()
{
  try {
    __env->Destroy();
    if(__logger)
      {__logger->log_debug("OpenRAVE Environment", "Environment destroyed");}
  } catch(const openrave_exception& e) {
    if(__logger)
      {__logger->log_warn("OpenRAVE Environment", "Could not destroy Environment. Ex:%s", e.what());}
  }
}

/** Lock the environment to prevent changes. */
void
OpenRaveEnvironment::lock()
{
  EnvironmentMutex::scoped_lock lock(__env->GetMutex());
}

/** Enable debugging messages of OpenRAVE. */
void
OpenRaveEnvironment::enable_debug()
{
  RaveSetDebugLevel(Level_Debug);
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
    __env->AddRobot(robot);
    if(__logger)
      {__logger->log_debug("OpenRAVE Environment", "Robot added to environment.");}
  } catch(openrave_exception &e) {
    if(__logger)
      {__logger->log_debug("OpenRAVE Environment", "Could not add robot to environment. OpenRAVE error:%s", e.message().c_str());}
  }
}

/** Add a robot into the scene.
 * @param filename path to robot's xml file
 * @return 1 if succeeded, 0 if not able to load file
 */
void
OpenRaveEnvironment::add_robot(const std::string& filename)
{
  // load the robot
  RobotBasePtr robot = __env->ReadRobotXMLFile(filename);

  // if could not load robot file: Check file path, and test file itself for correct syntax and semantics
  // by loading it directly into openrave with "openrave robotfile.xml"
  if( !robot )
    {throw fawkes::IllegalArgumentException("OpenRAVE Environment: Robot could not be loaded. Check xml file/path.");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Robot loaded.");}

  add_robot(robot);
}

/** Add a robot into the scene.
 * @param robot pointer to OpenRaveRobot object of robot to add
 * @return 1 if succeeded, 0 if not able to add robot
 */
void
OpenRaveEnvironment::add_robot(OpenRaveRobot* robot)
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
  try {
    boost::thread thviewer(boost::bind(SetViewer,__env,"qtcoin"));
  } catch( const openrave_exception &e) {
    if(__logger)
      {__logger->log_error("OpenRAVE Environment", "Could not load viewr. Ex:%s", e.what());}
    throw;
  }

  __viewer_enabled = true;
}

/** Autogenerate IKfast IK solver for robot.
 * @param robot pointer to OpenRaveRobot object
 * @param iktype IK type of solver (default: Transform6D; use TranslationDirection5D for 5DOF arms)
 */
void
OpenRaveEnvironment::load_IK_solver(OpenRaveRobot* robot, OpenRAVE::IkParameterization::Type iktype)
{
  RobotBasePtr robotBase = robot->get_robot_ptr();

  std::stringstream ssin,ssout;
  ssin << "LoadIKFastSolver " << robotBase->GetName() << " " << (int)iktype;
  // if necessary, add free inc for degrees of freedom
  //ssin << " " << 0.04f;
  if( !__mod_ikfast->SendCommand(ssout,ssin) )
    {throw fawkes::Exception("OpenRAVE Environment: Could not load ik solver");}
}

/** Plan collision-free path for current and target manipulator
 * configuration of a OpenRaveRobot robot.
 * @param robot pointer to OpenRaveRobot object of robot to use
 * @param sampling sampling time between each trajectory point (in seconds)
 */
void
OpenRaveEnvironment::run_planner(OpenRaveRobot* robot, float sampling)
{
  bool success;

  // init planner
  success = __planner->InitPlan(robot->get_robot_ptr(),robot->get_planner_params());
  if(!success)
    {throw fawkes::Exception("OpenRAVE Environment: Planner: init failed");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Planner: initialized");}

  // plan path
  boost::shared_ptr<Trajectory> traj(RaveCreateTrajectory(__env, robot->get_robot_ptr()->GetActiveDOF()));
  traj->Clear();
  success = __planner->PlanPath(traj);
  if(!success)
    {throw fawkes::Exception("OpenRAVE Environment: Planner: planning failed");}
  else if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Planner: path planned");}

  // re-timing the trajectory with cubic interpolation
  traj->CalcTrajTiming(robot->get_robot_ptr(),TrajectoryBase::CUBIC,true,true);

  // sampling trajectory
  std::vector<TrajectoryBase::TPOINT> points;
  for(dReal time = 0; time <= traj->GetTotalDuration(); time += (dReal)sampling) {
    TrajectoryBase::TPOINT point;
    traj->SampleTrajectory(time,point);
    points.push_back(point);
  }

  // setting robots trajectory
  std::vector< std::vector<dReal> >* trajRobot = robot->get_trajectory();
  trajRobot->clear();
  for(std::vector<TrajectoryBase::TPOINT>::iterator it = points.begin(); it!=points.end(); ++it) {
    trajRobot->push_back((*it).q);
  }

  // viewer options
  if( __viewer_enabled ) {

    // display trajectory in viewer
    __graph_handle.clear(); // remove all GraphHandlerPtr and currently drawn plots
    {
      RobotBasePtr tmp_robot = robot->get_robot_ptr();
      RobotBase::RobotStateSaver saver(tmp_robot); // save the state, do not modifiy currently active robot!
        for(std::vector<TrajectoryBase::TPOINT>::iterator it = points.begin(); it!=points.end(); ++it) {
          tmp_robot->SetActiveDOFValues((*it).q);
          __graph_handle.push_back(__env->plot3(RaveVector<float>(tmp_robot->GetActiveManipulator()->GetEndEffectorTransform().trans), 1, 0, 2.f, Vector(1.f, 0.f, 0.f, 1.f)));
        }
     } // robot state is restored

    // display motion in viewer
    if( robot->display_planned_movements())
      {robot->get_robot_ptr()->SetActiveMotion(traj);}
  }

}


/** Run graspplanning script for a given target.
 * Script loads grasping databse, checks if target is graspable for various grasps
 * and on success returns a string containing trajectory data.
 * Currently the grasping databse can only be accessed via python, so we use a short
 * python script (shortened and modified from officiel OpenRAVE graspplanning.py) to do the work.
 * @param target_name name of targeted object (KinBody)
 * @param robot pointer to OpenRaveRobot object of robot to use
 */
void
OpenRaveEnvironment::run_graspplanning(const std::string& target_name, OpenRaveRobot* robot)
{
  std::string filename = std::string(BINDIR) + "/../fawkes/src/plugins/openrave/python/graspplanning.py";
  std::string funcname = "runGrasp";

  boost::shared_ptr<Trajectory> traj(RaveCreateTrajectory(__env, robot->get_robot_ptr()->GetActiveDOF()));

  FILE* py_file = fopen(filename.c_str(), "r");
  if (py_file == NULL)
    {throw fawkes::Exception("OpenRAVE Environment: Graspplanning: opening python file failed");}

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
    throw fawkes::Exception("OpenRAVE Environment: Graspplanning: Python reference '__main__' does not exist.");
  }
  PyObject* py_dict = PyModule_GetDict(py_main);      // borrowed reference
  if( !py_dict ) {
    // __main__ should have a dictionary
    fclose(py_file);
    Py_Finalize();
    throw fawkes::Exception("OpenRAVE Environment: Graspplanning: Python reference '__main__' does not have a dictionary.");
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
          throw fawkes::Exception("OpenRAVE Environment: Graspplanning: No grasping path found.");
        }
        std::stringstream resval;
        resval << PyString_AsString(py_value);
        if (!traj->Read(resval, robot->get_robot_ptr()) ) {
          Py_DECREF(py_value);
          Py_DECREF(py_func);
          Py_Finalize();
          throw fawkes::Exception("OpenRAVE Environment: Graspplanning: Reading trajectory data failed.");
        }
        Py_DECREF(py_value);
      } else { // if calling function failed
        Py_DECREF(py_func);
        PyErr_Print();
        Py_Finalize();
        throw fawkes::Exception("OpenRAVE Environment: Graspplanning: Calling function failed.");
      }
    } else { // if loading func failed
      if (PyErr_Occurred())
        PyErr_Print();
      Py_XDECREF(py_func);
      Py_Finalize();
      throw fawkes::Exception("OpenRAVE Environment: Graspplanning: Loading function failed.");
    }
    Py_XDECREF(py_func);
  } else { // if loading module failed
    PyErr_Print();
    Py_Finalize();
    throw fawkes::Exception("OpenRAVE Environment: Graspplanning: Loading python file failed.");
  }

  Py_EndInterpreter(int_state); // close sub-interpreter
  PyThreadState_Swap(cur_state); // re-set active state to previous one
  PyGILState_Release(gil_state); // release GIL

  Py_Finalize(); // should be careful with that, as it closes global interpreter; Other threads running python may fail

  if(__logger)
    {__logger->log_debug("OpenRAVE Environment", "Graspplanning: path planned");}

  // re-timing the trajectory with cubic interpolation
  traj->CalcTrajTiming(robot->get_robot_ptr(),TrajectoryBase::CUBIC,true,true);

  // setting robots trajectory
  std::vector<TrajectoryBase::TPOINT> points = traj->GetPoints();
  std::vector< std::vector<dReal> >* trajRobot = robot->get_trajectory();
  trajRobot->clear();

  for(std::vector<TrajectoryBase::TPOINT>::iterator it = points.begin(); it!=points.end(); ++it) {
    trajRobot->push_back((*it).q);
  }

  // viewer options
  if( __viewer_enabled ) {

    // display trajectory in viewer
    __graph_handle.clear(); // remove all GraphHandlerPtr and currently drawn plots
    {
      RobotBasePtr tmp_robot = robot->get_robot_ptr();
      RobotBase::RobotStateSaver saver(tmp_robot); // save the state, do not modifiy currently active robot!
        for(std::vector<TrajectoryBase::TPOINT>::iterator it = points.begin(); it!=points.end(); ++it) {
          tmp_robot->SetActiveDOFValues((*it).q);
          __graph_handle.push_back(__env->plot3(RaveVector<float>(tmp_robot->GetActiveManipulator()->GetEndEffectorTransform().trans), 1, 0, 2.f, Vector(1.f, 0.f, 0.f, 1.f)));
        }
     } // robot state is restored

    // display motion in viewer
    if( robot->display_planned_movements()) {
      if (robot->get_robot_ptr()->GetActiveDOF() == traj->GetDOF())
        robot->get_robot_ptr()->SetActiveMotion(traj);
      else
        robot->get_robot_ptr()->SetMotion(traj);
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
    KinBodyPtr kb = __env->ReadKinBodyXMLFile(filename);
    kb->SetName(name);
    __env->AddKinBody(kb);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not add Object '%s'. Ex:%s", name.c_str(), e.what());
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
    KinBodyPtr kb = __env->GetKinBody(name);
    __env->Remove(kb);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not delete Object '%s'. Ex:%s", name.c_str(), e.what());
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
    KinBodyPtr kb = __env->GetKinBody(name);
    kb->SetName(new_name);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not rename Object '%s' to '%s'. Ex:%s", name.c_str(), new_name.c_str(), e.what());
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
 * @param robot if given, move relatively to robot (in most simple cases robot is at position (0,0,0) anyway, so this has no effect)
 * @return true if successful
 */
bool
OpenRaveEnvironment::move_object(const std::string& name, float trans_x, float trans_y, float trans_z, OpenRaveRobot* robot)
{
  try {
    KinBodyPtr kb = __env->GetKinBody(name);

    Transform transform = kb->GetTransform();
    transform.trans = Vector(trans_x, trans_y, trans_z);

    if( robot ) {
      Transform robotTrans = robot->get_robot_ptr()->GetTransform();
      transform.trans += robotTrans.trans;
    }

    kb->SetTransform(transform);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not move Object '%s'. Ex:%s", name.c_str(), e.what());
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
  try {
    KinBodyPtr kb = __env->GetKinBody(name);

    Vector q1 = quatFromAxisAngle(Vector(rot_x, 0.f, 0.f));
    Vector q2 = quatFromAxisAngle(Vector(0.f, rot_y, 0.f));
    Vector q3 = quatFromAxisAngle(Vector(0.f, 0.f, rot_z));

    Vector q12  = quatMultiply (q1, q2);
    Vector quat = quatMultiply (q12, q3);

    Transform transform = kb->GetTransform();
    transform.rot = quat;

    kb->SetTransform(transform);
  } catch(const OpenRAVE::openrave_exception &e) {
    if(__logger)
      __logger->log_warn("OpenRAVE Environment", "Could not rotate Object '%s'. Ex:%s", name.c_str(), e.what());
    return false;
  }

  return true;
}

} // end of namespace fawkes
