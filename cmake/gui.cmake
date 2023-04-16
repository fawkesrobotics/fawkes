pkg_check_modules(GTKMM gtkmm-3.0>=3.0.0)
if(NOT GTKMM_FOUND)
  pkg_check_modules(GTKMM gtkmm-2.4>=2.12)
endif()
remember_dependency(GTKMM)

function(depend_on_gui target)
  depend_on_pkgconfig_libs(${target} "cairomm-1.0;GTKMM;gthread-2.0")
  optional_depend_on_pkgconfig_libs(${target} giomm-2.4 giomm_dep_found)
  optional_depend_on_pkgconfig_libs(${target} gconfmm-2.6 gconfmm_dep_found)
  optional_depend_on_pkgconfig_libs(${target} glibmm-2.4 glibmm_dep_found)
  target_compile_options(
    ${target} PUBLIC -DHAVE_GTKMM -DGLIB_VERSION_MIN_REQUIRED=GLIB_VERSION_2_44
                     -DGLIB_VERSION_MAX_ALLOWED=GLIB_VERSION_2_60)
endfunction()
