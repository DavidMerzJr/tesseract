find_package(GTest REQUIRED)
find_package(tesseract_support REQUIRED)

# OMPL Planning Test/Example Program
add_executable(${PROJECT_NAME}_ompl_unit ompl_planner_tests.cpp)
target_link_libraries(${PROJECT_NAME}_ompl_unit ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME}_ompl tesseract::tesseract_support)
target_compile_options(${PROJECT_NAME}_ompl_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_ompl_unit PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_ompl_unit PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_ompl_unit PRIVATE ${GTEST_INCLUDE_DIRS})
if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${PROJECT_NAME}_ompl_unit "" AUTO)
else()
    gtest_discover_tests(${PROJECT_NAME}_ompl_unit)
endif()
add_dependencies(run_tests ${PROJECT_NAME}_ompl_unit)