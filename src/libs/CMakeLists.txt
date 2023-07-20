# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

add_subdirectory(core)
add_subdirectory(utils)
add_subdirectory(logging)
add_subdirectory(interface)
add_subdirectory(netcomm)
add_subdirectory(blackboard)
add_subdirectory(interfaces)
add_subdirectory(config)
add_subdirectory(tf)
add_subdirectory(plugin)
add_subdirectory(lua)
add_subdirectory(aspect)
add_subdirectory(network_logger)
add_subdirectory(webview)
add_subdirectory(gui_utils)
add_subdirectory(baseapp)
add_subdirectory(navgraph)
add_subdirectory(fvutils)
add_subdirectory(fvcams)
add_subdirectory(fvmodels)
add_subdirectory(fvfilters)
add_subdirectory(fvclassifiers)
add_subdirectory(fvstereo)
add_subdirectory(fvwidgets)
add_subdirectory(kdl_parser)
add_subdirectory(protobuf_clips)
add_subdirectory(pcl_utils)
add_subdirectory(syncpoint)
# add_subdirectory(googletest) empty
add_subdirectory(protoboard)
add_subdirectory(execution_time_estimator)
add_subdirectory(clips_gym)