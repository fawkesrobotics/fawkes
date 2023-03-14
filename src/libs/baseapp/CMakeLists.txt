add_library(
  fawkesbaseapp SHARED thread_manager.cpp timing_thread.cpp main_thread.cpp
                       daemonize.cpp init_options.cpp run.cpp)
find_package(Boost REQUIRED COMPONENTS system)
target_link_libraries(fawkesbaseapp fawkesnetcomm fawkesnetwork_logger
                      ${Boost_SYSTEM_LIBRARY})
target_compile_options(
  fawkesbaseapp
  PUBLIC -DHAVE_NETWORK_MANAGER
         -DHAVE_NETWORK_LOGGER
         -DHAVE_CONFIG_NETWORK_HANDLER
         -DHAVE_PLUGIN_NETWORK_HANDLER
         -DHAVE_LOGGING_FD_REDIRECT
         -DHAVE_BLACKBOARD
         -DHAVE_BOOST
         -DHAVE_CPP11)
if(TF_FOUND)
  depend_on_tf(fawkesbaseapp)
endif()
