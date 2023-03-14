pkg_check_modules(GTKMM gtkmm-3.0>=3.0.0)
if(NOT GTKMM_FOUND)
  pkg_check_modules(GTKMM gtkmm-2.4>=2.12)
endif()
pkg_check_modules(GIOMM giomm-2.4)
pkg_check_modules(CAIROMM cairomm-1.0)
pkg_check_modules(GCONFMM gconfmm-2.6)
pkg_check_modules(GLIBMM glibmm-2.4)
pkg_check_modules(GTHREAD gthread-2.0)

if(GTKMM_FOUND
   AND CAIROMM_FOUND
   AND GTHREAD_FOUND)
  set(HAVE_GUI 1)
endif()

if(NOT GTKMM_FOUND)
  message(WARNING "Omitting Gtk dependent GUI apps (gtkmm30[-devel] not found)")
endif()
if(NOT GIOMM_FOUND)
  message(WARNING "Omitting Gio dependent GUI apps")
endif()
if(NOT CAIROMM_FOUND)
  message(
    WARNING "Omitting Cairo dependent GUI apps (cairomm[-devel] not found)")
endif()
if(NOT GCONFMM_FOUND)
  message(
    WARNING "Omitting GConf dependent GUI apps (gconfmm26[-devel] not found)")
endif()
if(NOT GTHREAD_FOUND)
  message(WARNING "Omitting GThread dependent GUI apps (glib2-devel not found)")
endif()

function(depend_on_gui target)
  target_link_libraries(
    ${target}
    ${GTKMM_LDFLAGS}
    ${GIOMM_LDFLAGS}
    ${CAIROMM_LDFLAGS}
    ${GCONFMM_LDFLAGS}
    ${GLIBMM_LDFLAGS}
    ${GTHREAD_LDFLAGS})
  target_compile_options(
    ${target}
    PUBLIC ${GTKMM_CFLAGS}
           ${GIOMM_CFLAGS}
           ${CAIROMM_CFLAGS}
           ${GCONFMM_CFLAGS}
           ${GLIBMM_CFLAGS}
           ${GTHREAD_CFLAGS}
           -DHAVE_GTKMM
           -DGLIB_VERSION_MIN_REQUIRED=GLIB_VERSION_2_44
           -DGLIB_VERSION_MAX_ALLOWED=GLIB_VERSION_2_60)
endfunction()
