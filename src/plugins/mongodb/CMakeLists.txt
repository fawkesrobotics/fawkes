set(PLUGIN_mongodb
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)
add_subdirectory(aspect)

find_package(Boost COMPONENTS system filesystem)

set(PLUGIN_mongodb-rrd
    ON
    CACHE BOOL ON)

if(PLUGIN_mongodb OR PLUGIN_mongodb-rrd)
  if(mongocxx_FOUND)
    if(CPP_11_FOUND)
      if(Boost_FOUND)
        if(PLUGIN_mongodb)
          add_library(
            mongodb MODULE
            mongodb_plugin.cpp mongodb_thread.cpp mongodb_client_config.cpp
            mongodb_instance_config.cpp mongodb_replicaset_config.cpp utils.cpp)
          depend_on_mongodb(mongodb)

          target_link_libraries(
            mongodb
            fawkescore
            fawkesaspects
            fawkesmongodbaspect
            fawkesblackboard
            MongoDBManagedReplicaSetInterface
            pthread
            ${Boost_LIBRARIES})
          target_compile_features(mongodb PUBLIC cxx_std_11)
          target_include_directories(mongodb PUBLIC ${Boost_INCLUDE_DIRS})
        endif()
        if(PLUGIN_mongodb-rrd)
          add_library(mongodb-rrd MODULE mongorrd_plugin.cpp
                                         mongorrd_thread.cpp)

          depend_on_mongodb(mongodb-rrd)

          target_link_libraries(
            mongodb-rrd
            fawkescore
            fawkesutils
            fawkesaspects
            fawkesblackboard
            fawkesinterface
            fawkesconfig
            fawkesmongodbaspect
            fawkesrrdaspect
            pthread
            ${Boost_LIBRARIES})
          target_compile_features(mongodb-rrd PUBLIC cxx_std_11)
          target_include_directories(mongodb-rrd PUBLIC ${Boost_LIBRARIES})
        endif()
      else()
        build_skipped_message(mongodb "Boost system and fileystem")
      endif()

    else()
      build_skipped_message(mongodb "C++ 11")
    endif()
  else()
    build_skipped_message(mongodb "mongo-cxx-driver[-devel]")
  endif()

else()
  plugin_disabled_message("mongodb")
endif()
