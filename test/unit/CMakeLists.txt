set(test_sources main.tst.cpp extractors.tst.cpp event_scheduler.tst.cpp)
set(TEST_EXEC ${PROJECT_NAME}_unit_tests)

add_executable(${TEST_EXEC} ${test_sources})
target_link_libraries(${TEST_EXEC} PRIVATE ${PROJECT_NAME} gmock gtest)

if(DCOL_ENABLE_CODE_COVERAGE)
    setup_target_for_coverage_gcovr_xml(
        NAME
        ${PROJECT_NAME}_coverage
        EXECUTABLE
        ${TEST_EXEC}
        EXCLUDE
        ${PROJECT_SOURCE_DIR}/dep
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_BINARY_DIR})
endif()
