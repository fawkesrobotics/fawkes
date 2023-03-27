if(ROS_FOUND)
  add_library(fawkesrosaspect SHARED ros.cpp ros_inifin.cpp)
  target_link_libraries(fawkesrosaspect stdc++ fawkescore fawkesaspects
                        fawkesutils)
  depend_on_ros(fawkesrosaspect)
else()
  message(WARNING "Omitting ROS aspect (ROS dependency missing)")
endif()
