cmake_minimum_required(VERSION 2.8.3)
project(tf2_manif)

# Check that the compoiler has c++11 support
# Crash otherwise.
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} "
    "has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2
)

find_package(manif REQUIRED)

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS manif
)

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(tf2_manif-test test/tf2_manif-test.cpp)
  target_link_libraries(tf2_manif-test
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    MANIF::manif)

  # Set required C++11 flag
  if(CMAKE_VERSION VERSION_LESS "3.1")
    set_target_properties(tf2_manif-test PROPERTIES COMPILE_FLAGS "-std=c++11")
  else()
    set_property(TARGET tf2_manif-test PROPERTY CXX_STANDARD 11)
    set_property(TARGET tf2_manif-test PROPERTY CXX_STANDARD_REQUIRED ON)
    set_property(TARGET tf2_manif-test PROPERTY CXX_EXTENSIONS OFF)
  endif()

endif()
