add_custom_target(
  deploy
  npm install
  COMMAND npm run build
  COMMAND mkdir -p ${FAWKES_RES_DIR}/webview
  COMMAND cp -a dist/* ${FAWKES_RES_DIR}/webview/
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target(
  clean-deploy
  rm -rd ${FAWKES_RES_DIR}/webview/* || true
  COMMAND rm -rd ${CMAKE_CURRENT_SOURCE_DIR}/node_modules || true)
