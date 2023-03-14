set(fawkesaspects_SOURCES)
list(
  APPEND
  fawkesaspects_SOURCES
  syncpoint.cpp
  logging.cpp
  plugin_director.cpp
  vision.cpp
  syncpoint_manager.cpp
  fawkes_network.cpp
  aspect_provider.cpp
  thread_producer.cpp
  time_source.cpp
  blocked_timing/executor.cpp
  configurable.cpp
  manager.cpp
  inifins/syncpoint.cpp
  inifins/logging.cpp
  inifins/plugin_director.cpp
  inifins/vision.cpp
  inifins/syncpoint_manager.cpp
  inifins/fawkes_network.cpp
  inifins/aspect_provider.cpp
  inifins/inifin.cpp
  inifins/thread_producer.cpp
  inifins/time_source.cpp
  inifins/configurable.cpp
  inifins/blocked_timing.cpp
  inifins/blackboard.cpp
  inifins/vision_master.cpp
  inifins/clock.cpp
  inifins/mainloop.cpp
  inifins/network.cpp
  inifins/logger.cpp
  blocked_timing.cpp
  aspect.cpp
  mainloop/employer.cpp
  blackboard.cpp
  vision_master.cpp
  clock.cpp
  mainloop.cpp
  network.cpp
  logger.cpp)

if(PCL_FOUND)
  list(APPEND fawkesaspects_SOURCES pointcloud.cpp inifins/pointcloud.cpp)
endif()

if(TF_FOUND)
  list(APPEND fawkesaspects_SOURCES tf.cpp inifins/tf.cpp)
endif()

set(WEBVIEW_FOUND 0)
check_webview_deps(WEBVIEW_FOUND)
if(WEBVIEW_FOUND)
  list(APPEND fawkesaspects_SOURCES webview.cpp inifins/webview.cpp)
endif()

add_library(fawkesaspects SHARED ${fawkesaspects_SOURCES})
if(TF_FOUND)
  target_compile_options(fawkesaspects PUBLIC ${TF_CFLAGS})
  target_link_libraries(fawkesaspects ${TF_LDFLAGS} fawkestf)
endif()
if(PCL_FOUND)
  depend_on_pcl(fawkesaspects)
  target_link_libraries(fawkesaspects fawkespcl_utils)
endif()
if(WEBVIEW_FOUND)
  depend_on_webview(fawkesaspects)
  target_link_libraries(fawkesaspects fawkeswebview)
endif()
target_link_libraries(
  fawkesaspects
  fawkescore
  fawkesutils
  fawkesblackboard
  fawkesnetcomm
  fawkesconfig
  fawkesplugin
  fawkeslogging
  fawkessyncpoint)
