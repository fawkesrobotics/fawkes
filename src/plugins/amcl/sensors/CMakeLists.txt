add_library(fawkes_amcl_sensors SHARED amcl_sensor.cpp amcl_odom.cpp
                                       amcl_laser.cpp)
target_link_libraries(fawkes_amcl_sensors m fawkescore fawkes_amcl_pf
                      fawkes_amcl_map)
