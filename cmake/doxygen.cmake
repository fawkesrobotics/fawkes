find_package(Doxygen)
set(DOXYGEN_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/doc/api/)
set(DOXYGEN_WARN_LOGFILE doxywarnings.txt)
set(DOXYGEN_HAVE_DOT NO)
set(DOXYGEN_EXCLUDE_PATTERNS
    "${FAWKES_CORE_DIR}/src/plugins/webview/frontend;**/*.pb.*;\
    **/*Interface.*;**/*.py;**/attic/*;")
doxygen_add_docs(doxygen ${PROJECT_SOURCE_DIR} COMMENT "Generate man pages")
