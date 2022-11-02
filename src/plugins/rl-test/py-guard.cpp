/***************************************************************************
 *  py-guard.cpp -
 *
 *  Created:
 *  Copyright
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

#include "py-guard.h"

#include <list>
#include <map>
#include <regex>
#include <string>
using namespace py::literals;
#include <pybind11/numpy.h>

std::string
getConfigStringReplacedBasedirT(fawkes::Configuration *config, std::string configEntry)
{
	std::string config_value =
	  std::regex_replace(config->get_string(configEntry), std::regex("@BASEDIR@"), BASEDIR);
	return config_value;
}

PyGuard::PyGuard() //std::map<std::string, int> a_observation_space)
{
	py::initialize_interpreter();
	py_scope = py::module ::import("__main__").attr("__dict__");
	std::cout << "PyGuard initialized" << std::endl;
	std::cout << "PyGilState: " << PyGILState_Check() << std::endl;
	//loadConfig(config);
}

PyGuard *PyGuard::py_guard_instance{nullptr};

PyGuard *
PyGuard::getInstance()
{
	if (py_guard_instance == nullptr) {
		py_guard_instance = new PyGuard();
	}
	std::cout << "GetInstance: PyGilState: " << PyGILState_Check() << std::endl;
	return py_guard_instance;
}

void
PyGuard::loadConfig(fawkes::Configuration *config)
{
	std::string cfg_rl_agent_name = config->get_string("/rl-agent/name");
	std::string cfg_rl_agent_dir  = getConfigStringReplacedBasedirT(config, "/rl-agent/dir");

	cfg_executing_script       = config->get_string("/python/execution-script");
	std::string cfg_python_dir = getConfigStringReplacedBasedirT(config, "/python/dir");

	std::string cfg_env             = config->get_string("/rl-agent/env_name");
	std::string cfg_env_dir         = getConfigStringReplacedBasedirT(config, "/python/env-dir");
	std::string cfg_bin_plugins_dir = getConfigStringReplacedBasedirT(config, "/python/plugins-dir");
	std::cout << "Execution script: " << cfg_executing_script << std::endl;

	try {
		py::exec("import sys", py_scope);
		py::exec("print(\"Hello From python\")", py_scope);

		py::str sysPathAppend1 = (py::str)("sys.path.append(\"" + cfg_rl_agent_dir + "\")");
		py::str sysPathAppend2 = (py::str)("sys.path.append(\"" + cfg_python_dir + "\")");
		py::str sysPathAppend3 = (py::str)("sys.path.append(\"" + cfg_env_dir + "\")");
		py::str sysPathAppend4 = (py::str)("sys.path.append(\"" + cfg_bin_plugins_dir + "\")");

		py::exec(sysPathAppend1, py_scope);
		py::exec(sysPathAppend2, py_scope);
		py::exec(sysPathAppend3, py_scope);
		py::exec(sysPathAppend4, py_scope);
		py::exec("print(\"added config directories to sys.path\")");
		py::exec("print(sys.path)", py_scope);

		py::str dir_path = (py::str)("dir_path = \"" + cfg_rl_agent_dir + "\"");
		py::exec(dir_path, py_scope);

		py::str file_name =
		  (py::str)("file_name = \"" + cfg_rl_agent_dir + "/" + cfg_rl_agent_name + "\"");
		py::exec(file_name, py_scope);

	} catch (py::error_already_set &e) {
		py::module::import("traceback").attr("print_exception")(e.type(), e.value(), e.trace());
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << e.what() << std::endl;
	} catch (const std::runtime_error &re) {
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << re.what() << std::endl;
	} catch (...) {
		PyErr_Print();
		PyErr_Clear();
	}
	std::cout << "PyGuard finished load Config" << std::endl;
}

void
PyGuard::loadEnv()
{
	try {
		py::exec("print(\"PyGuard start loadEnv\")", py_scope);
		py::exec("print(sys.path)", py_scope);
		py::str env_name = (py::str)("env_name = \"Test\"");
		py::exec(env_name, py_scope);
		py::exec("print(env_name)", py_scope);

		py::exec("print(dir_path)", py_scope);
		py::exec("print(file_name)", py_scope);
		std::string path = py_scope["dir_path"].cast<std::string>();
		std::cout << "Init file: " << path << "/" << cfg_executing_script << std::endl;

		/* GILState_Check = 1 current thread is holding the GIL, 0 otherwise*/
		std::cout << "PyGilState: " << PyGILState_Check() << std::endl;
		//py::str training_script_path = (py::str)(rl_agent_dir + "/" + training_script);
		//py::gil_scoped_acquire gil;
		//auto gil_state = PyGILState_Ensure();
		py::str init_file = (py::str)(path + "/" + cfg_executing_script);
		py::eval_file(init_file, py_scope);
		//PyGILState_Release(gil_state);
		//py::gil_scoped_release release;

		//auto result = py::eval_file(init_file, py_scope);

		py::exec("print(\"PyGuard env created\")", py_scope);

		env   = py_scope["env"];
		model = py_scope["model"];
	} catch (py::error_already_set &e) {
		py::module::import("traceback").attr("print_exception")(e.type(), e.value(), e.trace());
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << e.what() << std::endl;
	} catch (const std::runtime_error &re) {
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << re.what() << std::endl;
	} catch (...) {
		PyErr_Print();
		PyErr_Clear();
	}

	std::cout << "PyGuard finished load Env" << std::endl;
}

void
PyGuard::print()
{
	py::exec("print(env_name)", py_scope);
	/*py::exec(R"(
		print("Obs dict: ", env.obs_dict)
	)",py_scope);*/

	std::cout << "PyGuard finished load print" << std::endl;
}

std::string
PyGuard::predict()
{
	std::string action_str = "";
	try {
		py::exec("print(\"PyGuard predict start\")", py_scope);
		//py_sope["obs"] = fact_string;
		py::exec("obs = env.getCurrentObs()", py_scope);
		py::exec("print(\"PyGuard obs: \",obs)", py_scope);

		py::exec(
		  "action, _ = model.predict(obs, action_masks = env.action_masks(), deterministic=True)",
		  py_scope);
		py::exec("print(\"Predicted action: \", action)", py_scope);

		py::exec("goal = env.action_dict[action]", py_scope);
		py::exec("print(\"Predicted goal: \", goal)", py_scope);
		action_str = py_scope["goal"].cast<std::string>();

	} catch (py::error_already_set &e) {
		py::module::import("traceback").attr("print_exception")(e.type(), e.value(), e.trace());
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << e.what() << std::endl;
	} catch (const std::runtime_error &re) {
		std::cout << "PYTHON EXCEPTION:" << std::endl;
		std::cout << re.what() << std::endl;
	} catch (...) {
		PyErr_Print();
		PyErr_Clear();
	}
	std::cout << "PyGuard finished predict" << std::endl;
	return action_str;
}

PyGuard::~PyGuard()
{
	py::finalize_interpreter();
}