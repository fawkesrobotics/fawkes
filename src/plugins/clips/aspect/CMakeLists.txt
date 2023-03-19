add_library(
  fawkesclipsaspect SHARED
  clips.cpp
  clips_env_manager.cpp
  clips_feature.cpp
  clips_feature_inifin.cpp
  clips_inifin.cpp
  clips_manager.cpp
  clips_manager_inifin.cpp)

depend_on_clipsmm(fawkesclipsaspect)
target_link_libraries(fawkesclipsaspect stdc++ fawkescore fawkesaspects
                      fawkesutils fawkesbaseapp)
