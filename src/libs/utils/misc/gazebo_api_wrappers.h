
/***************************************************************************
 *  gazebo_api_wrappers.h - Gazebo Compatibility API wrappers
 *
 *  Created: Thu Jun 29 13:27:47 2017 +0200
 *  Copyright  2017  Victor Matare
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

#ifndef __UTILS_GAZEBO_API_WRAPPERS_H
#define __UTILS_GAZEBO_API_WRAPPERS_H

#include <gazebo/gazebo.hh>

// Welcome to the Gazebo API refactoring hell.

#if GAZEBO_MAJOR_VERSION >= 8

# define GZWRAP_SIM_TIME SimTime
# define GZWRAP_REAL_TIME RealTime
# define GZWRAP_RUNNING Running
# define GZWRAP_MODEL_BY_NAME ModelByName
# define GZWRAP_MODEL_BY_INDEX ModelByIndex
# define GZWRAP_MODEL_COUNT ModelCount
# define GZWRAP_WORLD_POSE WorldPose
# define GZWRAP_ENTITY_BY_NAME EntityByName
# define GZWRAP_SIM_TIME SimTime
# define GZWRAP_NAME Name
# define GZWRAP_BOUNDING_BOX BoundingBox
# define GZWRAP_LENGTH Length
# define GZWRAP_MODELS Models
# define GZWRAP_PHYSICS Physics
# define GZWRAP_BASE_BY_NAME BaseByName

# define GZWRAP_POS Pos()
# define GZWRAP_ROT Rot()
# define GZWRAP_EULER Euler()
# define GZWRAP_X X()
# define GZWRAP_Y Y()
# define GZWRAP_Z Z()
# define GZWRAP_W W()

# define GZWRAP_ROT_ROLL Rot().Roll()
# define GZWRAP_ROT_PITCH Rot().Pitch()
# define GZWRAP_ROT_YAW Rot().Yaw()

#else

# define GZWRAP_SIM_TIME GetSimTime
# define GZWRAP_REAL_TIME GetRealTime
# define GZWRAP_RUNNING GetRunning
# define GZWRAP_MODEL_BY_NAME GetModel
# define GZWRAP_MODEL_BY_INDEX GetModel
# define GZWRAP_MODEL_COUNT GetModelCount
# define GZWRAP_WORLD_POSE GetWorldPose
# define GZWRAP_ENTITY_BY_NAME GetEntity
# define GZWRAP_SIM_TIME GetSimTime
# define GZWRAP_NAME GetName
# define GZWRAP_BOUNDING_BOX GetBoundingBox
# define GZWRAP_LENGTH GetLength
# define GZWRAP_MODELS GetModels
# define GZWRAP_PHYSICS GetPhysicsEngine
# define GZWRAP_BASE_BY_NAME GetByName

# define GZWRAP_POS pos
# define GZWRAP_ROT rot
# define GZWRAP_EULER GetAsEuler()
# define GZWRAP_X x
# define GZWRAP_Y y
# define GZWRAP_Z z
# define GZWRAP_W w

# define GZWRAP_ROT_ROLL rot.GetRoll()
# define GZWRAP_ROT_PITCH rot.GetPitch()
# define GZWRAP_ROT_YAW rot.GetYaw()

#endif

#define GZWRAP_POS_X GZWRAP_POS.GZWRAP_X
#define GZWRAP_POS_Y GZWRAP_POS.GZWRAP_Y
#define GZWRAP_POS_Z GZWRAP_POS.GZWRAP_Z

#define GZWRAP_ROT_X GZWRAP_ROT.GZWRAP_X
#define GZWRAP_ROT_Y GZWRAP_ROT.GZWRAP_Y
#define GZWRAP_ROT_Z GZWRAP_ROT.GZWRAP_Z
#define GZWRAP_ROT_W GZWRAP_ROT.GZWRAP_W

# define GZWRAP_ROT_EULER_X GZWRAP_ROT.GZWRAP_EULER.GZWRAP_X
# define GZWRAP_ROT_EULER_Y GZWRAP_ROT.GZWRAP_EULER.GZWRAP_Y
# define GZWRAP_ROT_EULER_Z GZWRAP_ROT.GZWRAP_EULER.GZWRAP_Z

namespace gzwrap {

#if GAZEBO_MAJOR_VERSION >= 8
typedef ignition::math::Pose3d Pose3d;
typedef ignition::math::Vector3d Vector3d;
#else
typedef gazebo::math::Pose Pose3d;
typedef gazebo::math::Vector3 Vector3d;
#endif

} // namespace gazebo_wrappers

#endif // __UTILS_GAZEBO_API_WRAPPERS_H
