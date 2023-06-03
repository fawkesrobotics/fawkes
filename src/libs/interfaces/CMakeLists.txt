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

add_executable(
  ffifacegen
  generator/checker.cpp
  generator/parser.cpp
  generator/pseudomap.cpp
  generator/main.cpp
  generator/enum_constant.cpp
  generator/tolua_generator.cpp
  generator/constant.cpp
  generator/message.cpp
  generator/field.cpp
  generator/cpp_generator.cpp
  generator/digest.cpp)

depend_on_libxml(ffifacegen)
depend_on_libcrypto(ffifacegen)

target_link_libraries(
  ffifacegen
  stdc++
  fawkescore
  fawkesutils
  fawkesinterface
  fawkesinterface_tolua
  fawkesutils_tolua)

get_target_property(ffifacegen_skipped ffifacegen EXCLUDE_FROM_DEFAULT_BUILD)
if(NOT ffifacegen_skipped)
  foreach(
    interface
    Position2DTrackInterface.xml
    GripperInterface.xml
    HumanSkeletonInterface.xml
    GameStateInterface.xml
    KeyValueInterface.xml
    MotorInterface.xml
    Laser1080Interface.xml
    OpenraveRobotMemoryInterface.xml
    CameraControlInterface.xml
    LocalizationInterface.xml
    Position3DInterface.xml
    TransformInterface.xml
    FacialExpressionInterface.xml
    ProtobufPeerInterface.xml
    HumanoidMotionInterface.xml
    SoccerPenaltyInterface.xml
    NavPathInterface.xml
    Laser720Interface.xml
    SwitchInterface.xml
    SkillerInterface.xml
    BatteryInterface.xml
    FacerInterface.xml
    JoystickInterface.xml
    VisualDisplay2DInterface.xml
    JointInterface.xml
    Laser360Interface.xml
    LedInterface.xml
    EclipseDebuggerInterface.xml
    IMUInterface.xml
    NavigatorInterface.xml
    KickerInterface.xml
    DynamicReconfigureInterface.xml
    SpeechSynthInterface.xml
    ExitSimulationInterface.xml
    ObjectPositionInterface.xml
    TestInterface.xml
    SpeechRecognitionInterface.xml)
    generate_interface_from_xml(${interface})
  endforeach()
else()
  build_skipped_message(interfaces ffifacegen)
endif()
