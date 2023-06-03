# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

add_library(
  fawkeswebview SHARED
  page_footer_generator.cpp
  nav_manager.cpp
  url_manager.cpp
  request_manager.cpp
  reply.cpp
  page_header_generator.cpp
  request.cpp
  request_dispatcher.cpp
  redirect_reply.cpp
  server.cpp
  rest_api_manager.cpp
  access_log.cpp
  rest_api.cpp
  file_reply.cpp
  error_reply.cpp
  page_reply.cpp
  user_verifier.cpp)
depend_on_webview(fawkeswebview)
target_link_libraries(fawkeswebview fawkescore fawkesutils fawkeslogging)
