cmake_minimum_required(VERSION 3.18)
project(trail VERSION 0.1.0 LANGUAGES C CXX)
#######################################################################
# Build flags
#######################################################################
add_compile_options(-Wall -Wextra -Wno-unused-function -O3 -march=native)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#######################################################################
# Dependencies (ament + ROS 2 + 3rd‑party)
#######################################################################
find_package(ament_cmake          REQUIRED)
find_package(rclcpp               REQUIRED)
find_package(sensor_msgs          REQUIRED)
find_package(std_msgs             REQUIRED)
find_package(geometry_msgs        REQUIRED)
find_package(nav_msgs             REQUIRED)
find_package(pcl_ros              REQUIRED)
find_package(pcl_conversions      REQUIRED)
find_package(magic_enum           REQUIRED)
find_package(yaml-cpp             REQUIRED)
find_package(rosbag2_cpp          REQUIRED)
find_package(rosbag2_storage      REQUIRED)


# ─────────────────────────────────────────────────────────────────────
# PCL ↔ qhull alias (Ubuntu 22.04 libqhull-dev ships only Qhull::qhull_r)
# ─────────────────────────────────────────────────────────────────────
if(NOT TARGET QHULL::QHULL)
  find_library(QHULL_LIB qhull_r REQUIRED)
  add_library(QHULL::QHULL SHARED IMPORTED)
  set_target_properties(QHULL::QHULL PROPERTIES IMPORTED_LOCATION ${QHULL_LIB})
endif()

# Local third‑party CMake packages ────────────────────────────────────
set(THIRDPARTY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
set(tiny-viewer_DIR ${THIRDPARTY_ROOT}/ctraj/thirdparty/tiny-viewer-install/lib/cmake/tiny-viewer)
find_package(tiny-viewer REQUIRED)

set(ctraj_DIR ${THIRDPARTY_ROOT}/ctraj-install/lib/cmake/ctraj)
find_package(ctraj REQUIRED)
link_directories(${THIRDPARTY_ROOT}/ctraj-install/lib)
link_directories(${THIRDPARTY_ROOT}/ctraj/thirdparty/tiny-viewer-install/lib)


find_package(spdlog REQUIRED)    
find_package(fmt REQUIRED)

option(USE_SYSTEM_TSL-ROBIN-MAP "Use system pre-installed tsl_robin" ON)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)

include(${THIRDPARTY_ROOT}/find_dependencies.cmake)

# Disable PCL pre‑compile for custom point types
add_definitions(-DPCL_NO_PRECOMPILE)

#######################################################################
# Source files
#######################################################################
file(GLOB_RECURSE SRC_CORE   CONFIGURE_DEPENDS src/core/*.cpp)
file(GLOB_RECURSE SRC_CONFIG CONFIGURE_DEPENDS src/config/*.cpp)
file(GLOB_RECURSE SRC_SENSOR CONFIGURE_DEPENDS src/sensor/*.cpp)
file(GLOB_RECURSE SRC_RMS CONFIGURE_DEPENDS src/rms/*.cpp)
file(GLOB_RECURSE SRC_VOXEL CONFIGURE_DEPENDS src/voxelmap/*.cpp)
set(ALL_SRC ${SRC_CORE} ${SRC_CONFIG} ${SRC_SENSOR} ${SRC_RMS} ${SRC_VOXEL})

#######################################################################
# Core library (isolated from rosidl auto‑targets)
#######################################################################
add_library(trail_core SHARED ${ALL_SRC})

set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_include_directories(trail_core PUBLIC
  $<BUILD_INTERFACE:${PROJECT_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
  /usr/local/include/magic_enum)

ament_target_dependencies(trail_core PUBLIC
  rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs
  pcl_ros pcl_conversions rosbag2_cpp rosbag2_storage)

target_link_libraries(trail_core PUBLIC
  yaml-cpp
  ctraj
  tsl::robin_map
  TBB::tbb
  fmt::fmt
  spdlog::spdlog)

#######################################################################
# ROS 2 Interfaces (messages)
#######################################################################
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrailState.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

# Retrieve C++ typesupport target once (keyword style)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(trail_core PUBLIC ${cpp_typesupport_target})

#######################################################################
# Executables
#######################################################################
add_executable(trail_node src/main.cpp)

foreach(_exe trail_node)
  # 1) first link package dependencies (uses *plain* signature internally)
  ament_target_dependencies(${_exe}
    rclcpp sensor_msgs std_msgs geometry_msgs nav_msgs
    rosbag2_cpp rosbag2_storage)
  # 2) now use the *plain* signature consistently
  target_link_libraries(${_exe} trail_core ${cpp_typesupport_target} ctraj)
endforeach()

#######################################################################
# Installation
##################### ##################################################
install(
  DIRECTORY ${THIRDPARTY_ROOT}/ctraj-install/lib/
            ${THIRDPARTY_ROOT}/ctraj/thirdparty/tiny-viewer-install/lib/
  DESTINATION lib
)
install(TARGETS
  trail_core trail_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py" PATTERN "*.launch.xml")

set(DATA_DIRS config)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/param)
  list(APPEND DATA_DIRS param)
endif()

set(DATA_DIRS config dataset)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/param)
  list(APPEND DATA_DIRS param)
endif()

install(DIRECTORY ${DATA_DIRS}
        DESTINATION share/${PROJECT_NAME})
#######################################################################
foreach(tgt trail_node)
  set_target_properties(${tgt} PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../:\$ORIGIN/../../ctraj-install/lib")
endforeach()

ament_package()
