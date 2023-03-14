pkg_check_modules(OROCOS_KDL orocos-kdl)
if(NOT OROCOS_KDL_FOUND)
  pkg_check_modules(OROCOS_KDL orocos_kdl)
endif()

pkg_check_modules(URDFDOMHEADERS urdfdom_headers>=0.40)
if(URDFDOMHEADERS_FOUND)
  set(HAVE_URDFDOM_TYPES_H 1)
else()
  pkg_check_modules(URDFDOMHEADERS urdfdom_headers)
endif()
pkg_check_modules(URDFDOM urdfdom)
pkg_check_modules(TINYXML tinyxml)

function(check_kdl_parser_deps KDL_FOUND)
  if(OROCOS_KDL_FOUND
     AND URDFDOMHEADERS_FOUND
     AND URDFDOM_FOUND
     AND TINYXML_FOUND)
    set(KDL_FOUND
        1
        PARENT_SCOPE)
  else()
    set(KDL_FOUND
        0
        PARENT_SCOPE)
    if(NOT OROCOS_KDL_FOUND)
      message(WARNING "orocos-kdl[-devel] dependency missing")
    endif()
    if(NOT URDFDOMHEADERS_FOUND)
      message(WARNING "urdfdom-headers[-devel] dependency missing")
    endif()
    if(NOT URDFDOM_FOUND)
      message(WARNING "urdfdom[-devel] dependency missing")
    endif()
    if(NOT TINYXML_FOUND)
      message(WARNING "tinyxml[-devel] dependency missing")
    endif()
  endif()
endfunction()

function(depend_on_kdl_parser target)
  if(HAVE_URDFDOM_TYPES_H)
    target_compile_options(${target} PUBLIC -DHAVE_URDFDOM_TYPES_H)
  endif()
  target_link_libraries(
    ${target} ${OROCOS_KDL_LDFLAGS} ${URDFDOMHEADERS_LDFLAGS}
    ${URDFDOM_LDFLAGS} ${TINYXML_LDFLAGS})
  target_compile_options(
    ${target} PUBLIC ${OROCOS_KDL_CFLAGS} ${URDFDOMHEADERS_CFLAGS}
                     ${URDFDOM_CFLAGS} ${TINYXML_CFLAGS})
endfunction()
