# cmake_minimum_required(VERSION 3.0.2)
# project(ucoslam)

# ## Compile as C++11, supported in ROS Kinetic and newer
# # add_compile_options(-std=c++11)

# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#   # cv_bridge
#   # image_transport
#   # message_generation
#   # pcl_conversions
#   # pcl_ros
#   # rosbag
#   roscpp
#   # sensor_msgs
#   std_msgs
#   # std_srvs
# )

# ## System dependencies are found with CMake's conventions
# # find_package(Boost REQUIRED COMPONENTS system)


# ## Uncomment this if the package has a setup.py. This macro ensures
# ## modules and global scripts declared therein get installed
# ## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# # catkin_python_setup()

# ################################################
# ## Declare ROS messages, services and actions ##
# ################################################

# ## To declare and build messages, services or actions from within this
# ## package, follow these steps:
# ## * Let MSG_DEP_SET be the set of packages whose message types you use in
# ##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# ## * In the file package.xml:
# ##   * add a build_depend tag for "message_generation"
# ##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
# ##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# ##     but can be declared for certainty nonetheless:
# ##     * add a exec_depend tag for "message_runtime"
# ## * In this file (CMakeLists.txt):
# ##   * add "message_generation" and every package in MSG_DEP_SET to
# ##     find_package(catkin REQUIRED COMPONENTS ...)
# ##   * add "message_runtime" and every package in MSG_DEP_SET to
# ##     catkin_package(CATKIN_DEPENDS ...)
# ##   * uncomment the add_*_files sections below as needed
# ##     and list every .msg/.srv/.action file to be processed
# ##   * uncomment the generate_messages entry below
# ##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# ## Generate messages in the 'msg' folder
# # add_message_files(
# #   FILES
# #   Message1.msg
# #   Message2.msg
# # )

# ## Generate services in the 'srv' folder
# # add_service_files(
# #   FILES
# #   Service1.srv
# #   Service2.srv
# # )

# ## Generate actions in the 'action' folder
# # add_action_files(
# #   FILES
# #   Action1.action
# #   Action2.action
# # )

# ## Generate added messages and services with any dependencies listed here
# # generate_messages(
# #   DEPENDENCIES
# #   sensor_msgs#   std_msgs
# # )

# ################################################
# ## Declare ROS dynamic reconfigure parameters ##
# ################################################

# ## To declare and build dynamic reconfigure parameters within this
# ## package, follow these steps:
# ## * In the file package.xml:
# ##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
# ## * In this file (CMakeLists.txt):
# ##   * add "dynamic_reconfigure" to
# ##     find_package(catkin REQUIRED COMPONENTS ...)
# ##   * uncomment the "generate_dynamic_reconfigure_options" section below
# ##     and list every .cfg file to be processed

# ## Generate dynamic reconfigure parameters in the 'cfg' folder
# # generate_dynamic_reconfigure_options(
# #   cfg/DynReconf1.cfg
# #   cfg/DynReconf2.cfg
# # )

# ###################################
# ## catkin specific configuration ##
# ###################################
# ## The catkin_package macro generates cmake config files for your package
# ## Declare things to be passed to dependent projects
# ## INCLUDE_DIRS: uncomment this if your package contains header files
# ## LIBRARIES: libraries you create in this project that dependent projects also need
# ## CATKIN_DEPENDS: catkin_packages dependent projects also need
# ## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES ucoslam
# #  CATKIN_DEPENDS cv_bridge image_transport message_generation pcl_conversions pcl_ros rosbag roscpp sensor_msgs std_msgs std_srvs
# #  DEPENDS system_lib
# )

# ###########
# ## Build ##
# ###########

# ## Specify additional locations of header files
# ## Your package locations should be listed before other locations
# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )

# ## Declare a C++ library
# # add_library(${PROJECT_NAME}
# #   src/${PROJECT_NAME}/ucoslam.cpp
# # )

# ## Add cmake target dependencies of the library
# ## as an example, code may need to be generated before libraries
# ## either from message generation or dynamic reconfigure
# # add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ executable
# ## With catkin_make all packages are built within a single CMake context
# ## The recommended prefix ensures that target names across packages don't collide
# # add_executable(${PROJECT_NAME}_node src/ucoslam_node.cpp)

# ## Rename C++ executable without prefix
# ## The above recommended prefix causes long target names, the following renames the
# ## target back to the shorter version for ease of user use
# ## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# # set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# ## Add cmake target dependencies of the executable
# ## same as for the library above
# # add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Specify libraries to link a library or executable target against
# # target_link_libraries(${PROJECT_NAME}_node
# #   ${catkin_LIBRARIES}
# # )

# #############
# ## Install ##
# #############

# # all install targets should use catkin DESTINATION variables
# # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# ## Mark executable scripts (Python etc.) for installation
# ## in contrast to setup.py, you can choose the destination
# # catkin_install_python(PROGRAMS
# #   scripts/my_python_script
# #   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# # )

# ## Mark executables for installation
# ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# # install(TARGETS ${PROJECT_NAME}_node
# #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# # )

# ## Mark libraries for installation
# ## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# # install(TARGETS ${PROJECT_NAME}
# #   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# #   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# # )

# ## Mark cpp header files for installation
# # install(DIRECTORY include/${PROJECT_NAME}/
# #   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# #   FILES_MATCHING PATTERN "*.h"
# #   PATTERN ".svn" EXCLUDE
# # )

# ## Mark other files for installation (e.g. launch and bag files, etc.)
# # install(FILES
# #   # myfile1
# #   # myfile2
# #   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# # )

# #############
# ## Testing ##
# #############

# ## Add gtest based cpp test target and link libraries
# # catkin_add_gtest(${PROJECT_NAME}-test test/test_ucoslam.cpp)
# # if(TARGET ${PROJECT_NAME}-test)
# #   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# # endif()

# ## Add folders to be run by python nosetests
# # catkin_add_nosetests(test)
# add_executable(ucoslam_node src/ucoslam_node.cpp)
# target_link_libraries(ucoslam_node ${catkin_LIBRARIES})


### --------------------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.10)
project(ucoslam LANGUAGES CXX C)    # 针对使用catkin build 时候的报错修改 project(ucoslam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY) # 解决 Eigen3 编译时的对齐问题
# add_compile_definitions(EIGEN_DONT_ALIGN_STATICALLY)  # 解决 Eigen3 编译时的对齐问题
# add_definitions(-DEIGEN_MPL2_ONLY) # ??解决 Eigen3 编译时的对齐问题


add_compile_definitions(
  EIGEN_DONT_ALIGN_STATICALLY
  EIGEN_DONT_VECTORIZE
)  # 06.23 解决 Eigen3 编译时的对齐问题


# 设置编译选项，-g 用于生成调试信息 25.06.09
# add_compile_options(-g)
# 设置编译类型为 Debug，并添加调试信息  25.06.09
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")


# set(cv_bridge_DIR "/home/kilox/workspace/3rdparty/cv_bridge_noetic/share/cv_bridge/cmake")
# set(cv_bridge_DIR "/home/kilox/cloud_mapping/src/ucoslam2/3rdparty/cv_bridge_noetic")
# set(cv_bridge_DIR "/home/kilox/cloud_mapping/src/ucoslam2/3rdparty/cv_bridge_noetic/share/cv_bridge/cmake")

# set(OpenCV_DIR "/home/kilox/workspace/3rdparty/opencv-4.5.5/lib/cmake/opencv4") # add necessary in desktop kilox
# set(OpenCV_DIR "/home/kilox/cloud_mapping/src/ucoslam2/3rdparty/opencv-4.5.5/lib/cmake/opencv4") # add necessary in docker
# set(OpenCV_DIR "/home/kilox/cloud_mapping/src/ucoslam2/3rdparty/opencv-4.5.5/build")
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4") # 06.21


# set(PCL_DIR "/home/kilox/workspace/3rdparty/pcl-1.15/share/pcl-1.15") # 先不设置 06.20
# set(PCL_DIR "/home/kilox/cloud_mapping/src/ucoslam2/3rdparty/pcl-1.15/share/pcl-1.15")

find_package(OpenCV 4.2.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

# 重点：加入算法库目录 old_cmake 06.19
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ucoslam_or ${CMAKE_BINARY_DIR}/ucoslam_or_build)
# message(STATUS "Build add_subdirectory ucoslam_or")



# 告诉CMake到哪里找 ucoslamConfig.cmake
# set(ucoslam_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/share/ucoslam")
# find_package(ucoslam REQUIRED )
# message(STATUS "cv_bridge_DIR is---------------------------------------: ${cv_bridge_DIR}")
message(STATUS "OpenCV_DIR is---------------------------------------: ${OpenCV_DIR}")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  cv_bridge
  pcl_ros
  pcl_conversions
  sensor_msgs
  image_transport
  message_generation
  std_srvs
  std_msgs
  eigen_conversions
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosbag
    cv_bridge
    pcl_ros
    pcl_conversions
    sensor_msgs
    image_transport
    std_srvs
    std_msgs
    eigen_conversions
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
    # ${ucoslam_INCLUDE_DIRS}
)
# 移动位置
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Dev_ucoslam ${CMAKE_BINARY_DIR}/Dev_ucoslam_build)
message(STATUS "Build add_subdirectory Dev_ucoslam")

add_executable(ucoslam_node src/ucoslam_node.cpp)
target_link_libraries(ucoslam_node
    PRIVATE
    ucoslam
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    # ${ucoslam_LIBS}
)

install(TARGETS ucoslam_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


