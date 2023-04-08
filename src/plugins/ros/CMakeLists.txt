set(PLUGIN_ros
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-talkerpub
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-tf
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-pcl
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-images
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-laserscan
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-odometry
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-cmdvel
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-navigator
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-dynamic-reconfigure
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-navgraph-breakout
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-joint
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-robot-description
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-move-base
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-skiller
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-position-3d
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-clock
    ON
    CACHE BOOL ON)
set(PLUGIN_ros-imu
    ON
    CACHE BOOL ON)

add_subdirectory(aspect)
if(PLUGIN_ros)

  add_library(ros MODULE ros_plugin.cpp node_thread.cpp)
  depend_on_ros(ros)
  target_link_libraries(
    ros
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesrosaspect)
else()
  plugin_disabled_message(ros)
endif()

if(PLUGIN_ros-talkerpub)
  pkg_check_modules(std_msgs std_msgs)
  if(std_msgs_FOUND)

    add_library(ros-talkerpub MODULE talkerpub_plugin.cpp talkerpub_thread.cpp)
    depend_on_ros(ros-talkerpub)
    depend_on_extra_libs(ros-talkerpub std_msgs)
    target_link_libraries(
      ros-talkerpub
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect)
  endif()
else()
  plugin_disabled_message(ros-talkerpub)
endif()

if(PLUGIN_ros-tf)
  if(TF_FOUND)
    pkg_check_modules(tf tf)
    if(tf_FOUND)
      add_library(ros-tf MODULE tf_plugin.cpp tf_thread.cpp)
      depend_on_ros(ros-tf)
      depend_on_tf(ros-tf)
      depend_on_extra_libs(ros-tf tf)
      target_link_libraries(
        ros-tf
        fawkescore
        fawkesutils
        fawkesaspects
        fawkesblackboard
        fawkesinterface
        fawkesrosaspect
        fawkestf
        TransformInterface)
      pkg_check_modules(tf2 tf2)
      if(tf2_FOUND)
        depend_on_extra_libs(ros-tf tf2)
        target_compile_definitions(ros-tf PUBLIC HAVE_TF2_MSGS)
      else()
        message(
          WARNING
            "Omitting ros-tf tf2 support (ros-tf2[-devel] dependency missing)")
      endif()
    else()
      message(WARNING "Omitting ros-tf tf (ros-tf[-devel] dependency missing)")
    endif()
  else()
    message(WARNING "Omitting ros-tf (fawkestf dependency missing)")
  endif()
else()
  plugin_disabled_message(ros-tf)
endif()

pkg_check_modules(sensor_msgs sensor_msgs)
set(sensor_msgs_PLUGINS)
if(sensor_msgs_FOUND)
  if(PLUGIN_ros-pcl)
    list(APPEND sensor_msgs_PLUGINS ros-pcl)
    if(PCL_FOUND)
      pkg_check_modules(pcl_conversions pcl_conversions)
      if(pcl_conversions_FOUND)
        add_library(ros-pcl MODULE pcl_plugin.cpp pcl_thread.cpp)
        depend_on_ros(ros-pcl)
        depend_on_pcl(ros-pcl)
        depend_on_extra_libs(ros-pcl sensor_msgs)
        depend_on_extra_libs(ros-pcl pcl_conversions)
        target_link_libraries(
          ros-pcl
          fawkescore
          fawkesutils
          fawkesaspects
          fawkesblackboard
          fawkesinterface
          fawkesrosaspect
          fawkestf
          fawkespcl_utils)
      else()
        message(
          WARNING
            "Omitting ros-pcl (ros-pcl_conversions[-devel] dependency missing)")
      endif()
    else()
      message(WARNING "Omitting ros-pcl (pcl[-devel] dependency missing)")
    endif()
  else()
    plugin_disabled_message(ros-pcl)
  endif()

  if(PLUGIN_ros-images)
    list(APPEND sensor_msgs_PLUGINS ros-image)
    add_library(ros-images MODULE image_plugin.cpp image_thread.cpp)
    depend_on_ros(ros-images)
    depend_on_extra_libs(ros-images sensor_msgs)
    target_link_libraries(
      ros-images
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      fvutils)
  else()
    plugin_disabled_message(ros-images)
  endif()

  if(PLUGIN_ros-laserscan)
    list(APPEND sensor_msgs_PLUGINS ros-laserscan)
    add_library(ros-laserscan MODULE laserscan_plugin.cpp laserscan_thread.cpp)
    depend_on_ros(ros-laserscan)
    depend_on_extra_libs(ros-laserscan sensor_msgs)
    target_link_libraries(
      ros-laserscan
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      fvutils
      Laser360Interface
      Laser720Interface
      Laser1080Interface)
  else()
    plugin_disabled_message(ros-laserscan)
  endif()

  if(PLUGIN_ros-joint)
    list(APPEND sensor_msgs_PLUGINS ros-joint)
    add_library(ros-joint MODULE joint_plugin.cpp joint_thread.cpp)
    depend_on_ros(ros-joint)
    depend_on_extra_libs(ros-joint sensor_msgs)
    target_link_libraries(
      ros-joint
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      JointInterface)
  else()
    plugin_disabled_message(ros-joint)
  endif()

  if(PLUGIN_ros-imu)
    list(APPEND sensor_msgs_PLUGINS ros-imu)
    add_library(ros-imu imu_plugin.cpp imu_thread.cpp)
    depend_on_ros(ros-imu)
    depend_on_extra_libs(ros-imu sensor_msgs)
    target_link_libraries(
      ros-imu
      MODULE
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      IMUInterface)
  else()
    plugin_disabled_message(ros-imu)
  endif()
else()
  if(sensor_msgs_PLUGINS)
    message(
      WARNING
        "Omitting ${sensor_msgs_PLUGINS} (ros-sensor_msgs[-devel] dependency missing)"
    )
  endif()
endif()

if(PLUGIN_ros-odometry)
  pkg_check_modules(nav_msgs nav_msgs)
  if(nav_msgs_FOUND)
    add_library(ros-odometry MODULE odometry_plugin.cpp odometry_thread.cpp)
    depend_on_ros(ros-odometry)
    depend_on_extra_libs(ros-odometry nav_msgs)
    target_link_libraries(
      ros-odometry
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkestf
      fawkesrosaspect
      MotorInterface)
  else()
    message(
      WARNING "Omitting ros-odometry (ros-nav_msgs[-devel] dependency missing)")
  endif()
else()
  plugin_disabled_message(ros-odometry)
endif()

if(PLUGIN_ros-cmdvel)
  pkg_check_modules(geometry_msgs geometry_msgs)
  if(geometry_msgs_FOUND)
    add_library(ros-cmdvel MODULE cmdvel_thread.cpp cmdvel_plugin.cpp)
    depend_on_ros(ros-cmdvel)
    depend_on_extra_libs(ros-cmdvel geometry_msgs)
    target_link_libraries(
      ros-cmdvel
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      MotorInterface)
  else()
    message(
      WARNING
        "Omitting ros-cmdvel (ros-geometry_msgs[-devel] dependency missing)")
  endif()
else()
  plugin_disabled_message(ros-cmdvel)
endif()

if(PLUGIN_ros-dynamic-reconfigure)
  pkg_check_modules(dynamic_reconfigure dynamic_reconfigure)
  if(dynamic_reconfigure_FOUND)
    add_library(ros-dynamic-reconfigure MODULE dynamic_reconfigure_plugin.cpp
                                               dynamic_reconfigure_thread.cpp)
    depend_on_ros(ros-dynamic-reconfigure)
    depend_on_extra_libs(ros-dynamic-reconfigure dynamic_reconfigure)
    target_link_libraries(
      ros-dynamic-reconfigure
      m
      pthread
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      DynamicReconfigureInterface)
  else()
    message(
      WARNING
        "Omitting ros-dynamic-reconfigure (ros-dynamic_reconfigure[-devel] dependency missing)"
    )
  endif()
else()
  plugin_disabled_message(ros-dynamic-reconfigure)
endif()

set(ros-navigator_DEPS move_base_msgs dynamic_reconfigure actionlib)
check_deps_extra_libs(${ros-navigator_DEPS})
if(NOT EXTRA_LIBS_WARNING)

  if(PLUGIN_ros-navigator)
    find_package(Boost COMPONENTS thread)
    if(Boost_THREAD_FOUND)

      add_library(ros-navigator MODULE navigator_plugin.cpp
                                       navigator_thread.cpp)
      target_link_libraries(ros-navigator ${Boost_THREAD_LIBRARY})
      depend_on_ros(ros-navigator)
      depend_on_extra_libs(ros-navigator "${ros-navigator_DEPS}")
      target_link_libraries(
        ros-navigator
        m
        pthread
        fawkescore
        fawkesutils
        fawkesaspects
        fawkesblackboard
        fawkesinterface
        fawkesrosaspect
        NavigatorInterface)
    else()
      plugin_disabled_message(ros-navigator)
    endif()

    if(PLUGIN_ros-move-base)
      add_library(ros-move-base MODULE move_base_plugin.cpp
                                       move_base_thread.cpp)
      depend_on_ros(ros-move-base)
      depend_on_extra_libs(ros-move-base "${ros-navigator_DEPS}")
      target_link_libraries(ros-move-base ${Boost_THREAD_LIBRARY})
      target_link_libraries(
        ros-move-base
        fawkescore
        fawkesutils
        fawkesaspects
        fawkesblackboard
        fawkesinterface
        fawkestf
        fawkesrosaspect
        NavigatorInterface)
    else()
      plugin_disabled_message(ros-move-base)
    endif()

  else()
    message(
      WARNING
        "Omitting ros-navigator and ros-move-base (boost thread dependency missing)"
    )
  endif()
else()
  if(PLUGIN_ros-move-base OR PLUGIN_ros-navigator)
    message(WARNING "Omitting ros-navigator and ros-move-base")
    foreach(warn ${EXTRA_LIBS_WARNING})
      message(WARNING ${warn})
    endforeach()
  endif()
endif()

pkg_check_modules(fawkes_msgs fawkes_msgs)
if(fawkes_msgs_FOUND)
  if(PLUGIN_ros-navgraph-breakout)
    add_library(ros-navgraph-breakout MODULE navgraph_breakout_plugin.cpp
                                             navgraph_breakout_thread.cpp)
    depend_on_ros(ros-navgraph-breakout)
    depend_on_extra_libs(ros-navgraph-breakout fawkes_msgs)
    target_link_libraries(
      ros-navgraph-breakout
      m
      pthread
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      NavigatorInterface)
  else()
    plugin_disabled_message(ros-navgraph-breakout)
  endif()

  if(PLUGIN_ros-position-3d)
    add_library(ros-position-3d MODULE position_3d_thread.cpp
                                       position_3d_plugin.cpp)
    depend_on_ros(ros-position-3d)
    depend_on_extra_libs(ros-position-3d fawkes_msgs)
    target_link_libraries(
      ros-position-3d
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      Position3DInterface)
  else()
    plugin_disabled_message(ros-position-3d)
  endif()

  if(PLUGIN_ros-skiller)
    pkg_check_modules(actionlib actionlib)
    if(actionlib_FOUND)
      add_library(ros-skiller MODULE skiller_plugin.cpp skiller_thread.cpp)
      depend_on_ros(ros-skiller)
      depend_on_extra_libs(ros-skiller fawkes_msgs)
      depend_on_extra_libs(ros-skiller actionlib)
      target_link_libraries(
        ros-skiller
        fawkescore
        ros-navgraph-breakouts
        fawkesaspects
        fawkesblackboard
        fawkesinterface
        fawkesrosaspect
        SkillerInterface)
    else()
      message(
        WARNING
          "Omitting ros-skiller (ros-actionlib[-devel] dependency missing)")
    endif()
  else()
    plugin_disabled_message(ros-skiller)
  endif()

else()
  if(PLUGIN_ros-navgraph-breakout
     OR PLUGIN_ros-skiller
     OR PLUGIN_ros-position-3d)
    message(
      WARNING
        "Omitting ros-navgraph-breakout, ros-skiller and ros-position-3d (fawkes-msgs dependency missing)"
    )
  endif()
endif()

if(PLUGIN_ros-clock)
  pkg_check_modules(rosgraph_msgs rosgraph_msgs)
  if(rosgraph_msgs_FOUND)
    add_library(ros-clock clock_plugin.cpp clock_thread.cpp)
    depend_on_ros(ros-clock)
    depend_on_extra_libs(ros-clock rosgraph_msgs)
    target_link_libraries(
      ros-clock
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect)

  else()
    message(
      WARNING
        "Omitting ros-skiller (ros-rosgraph_msgs[-devel] dependency missing)")
  endif()
else()
  plugin_disabled_message(ros-clock)
endif()

if(PLUGIN_ros-robot-description)
  add_library(ros-robot-description MODULE robot_description_plugin.cpp
                                           robot_description_thread.cpp)
  depend_on_ros(ros-robot-description)
  target_link_libraries(ros-robot-description fawkescore fawkesutils
                        fawkesaspects fawkesrosaspect)

else()
  plugin_disabled_message(ros-robot-description)
endif()
