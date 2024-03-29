cmake_minimum_required(VERSION 3.1.3)
project(braccio_morpheus_braccio_ikfast_plugin)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf2_kdl
  tf2_eigen
  eigen_conversions
)
find_package(LAPACK REQUIRED)

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

catkin_package()

set(IKFAST_LIBRARY_NAME braccio_morpheus_braccio_moveit_ikfast_plugin)
add_library(${IKFAST_LIBRARY_NAME} src/braccio_morpheus_braccio_ikfast_moveit_plugin.cpp)
target_link_libraries(${IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
# suppress warnings about unused variables in OpenRave's solver code
target_compile_options(${IKFAST_LIBRARY_NAME} PRIVATE -Wno-unused-variable)

install(TARGETS
  ${IKFAST_LIBRARY_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(
  FILES
  braccio_morpheus_braccio_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
