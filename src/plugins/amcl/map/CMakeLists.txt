add_library(fawkes_amcl_map SHARED map.c map_cspace.cpp map_range.c map_store.c
                                   map_draw.c)
target_link_libraries(fawkes_amcl_map m)
