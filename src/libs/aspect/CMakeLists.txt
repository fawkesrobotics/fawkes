add_library(
  fawkesaspects SHARED
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
  target_sources(fawkesaspects PRIVATE pointcloud.cpp inifins/pointcloud.cpp)
  depend_on_pcl(fawkesaspects)
  target_link_libraries(fawkesaspects fawkespcl_utils)
endif()

optional_depend_on_pkgconfig_libs(fawkesaspects "${tf_deps}" tf_deps_found)
if(tf_deps_found)
  target_sources(fawkesaspects PRIVATE tf.cpp inifins/tf.cpp)
  depend_on_tf(fawkesaspects)
  target_link_libraries(fawkesaspects fawkestf)
endif()

optional_depend_on_pkgconfig_libs(fawkesaspects "${webview_deps}"
                                  webview_deps_found)
if(webview_deps_found)
  target_sources(fawkesaspects PRIVATE webview.cpp inifins/webview.cpp)
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
