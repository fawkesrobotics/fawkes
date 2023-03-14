include(libxmlpp)
include(libcrypto)
if(libxml_FOUND)
  if(ENCRYPTION_SUPPORT)

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
    # TODO automate tolua includes?!
    target_link_libraries(
      ffifacegen
      stdc++
      fawkescore
      fawkesutils
      fawkesinterface
      fawkesinterface_tolua
      fawkesutils_tolua)

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
      get_filename_component(if_name ${interface} NAME_WLE)
      add_custom_command(
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.tolua
               ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.h
               ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.cpp
        COMMAND ffifacegen -d ${CMAKE_CURRENT_SOURCE_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/${interface}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${interface})
      include(lua)
      generate_tolua(${if_name} ${if_name}.tolua)
      add_library(${if_name} SHARED ${if_name}_tolua.cpp
                                    ${CMAKE_CURRENT_SOURCE_DIR}/${if_name}.cpp)
      target_link_libraries(${if_name} fawkescore fawkesutils fawkesinterface)
      # target_link_libraries(fawkesutils_tolua fawkescore fawkesutils)
    endforeach()

  else()
    message(
      WARNING "Omitting interface generator (openssl[-devel] not installed)")
  endif()
else()
  message(
    WARNING "Omitting interface generator (libxml++[-devel] not installed)")
endif()
# $(SILENT)echo -e "$(INDENT_PRINT)--> $(TYELLOW)Interfaces cannot be
# generated$(TNORMAL)"
