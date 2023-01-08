/***************************************************************************
 *  py-guard.h -
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

#include <config/config.h>
#include <pybind11/embed.h>

#include <iostream>
#include <list>
#include <map>
#include <string>
namespace py = pybind11;

class PyGuard
{
public:
	static PyGuard *getInstance();
	void            loadConfig(fawkes::Configuration *config);
	void            loadEnv();
	void            print();
	std::string     predict();
	std::string     getGoalId();

protected:
	PyGuard();
	~PyGuard();

private:
	py::object py_scope;
	py::object env;
	py::object model;

	std::string cfg_executing_script;

	static PyGuard *py_guard_instance;
	//std::map<std::string, int> observation_space;
};
