add_custom_target(
  integration-test
  ${FAWKES_CORE_DIR}/etc/scripts/integration_test.sh ${PROJECT_SOURCE_DIR}
  COMMENT "Run the test scripts within tests.d/")
