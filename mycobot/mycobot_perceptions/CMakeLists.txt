# Minimum CMake version and project definition
cmake_minimum_required(VERSION 3.8)

# Set CMake policies
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(mycobot_perceptions)

# Build settings
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Prefer shared libraries over static ones. Without this, CMake finds the
# static /usr/local/lib/libusb-1.0.a that is not built with -fPIC, which makes
# the final shared object fail to link. Restricting to .so forces CMake to
# skip that static archive and use the system shared libusb instead.
set(CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".dylib" ".dll")

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Define required packages (alphabetical order)
# Note: moveit_core, moveit_ros_planning, moveit_ros_planning_interface and
# moveit_task_constructor_core are needed because the imported targets exported
# by mycobot_mtc_pick_place_demo transitively link against
# moveit_core::moveit_collision_detection and
# moveit_task_constructor_core::moveit_task_constructor_core_stages. Without
# finding these packages first, those ::-prefixed imported targets are unknown
# to CMake and the build fails at config time.
set(REQUIRED_PACKAGES
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  mycobot_interfaces
  mycobot_mtc_pick_place_demo
  PCL
  pcl_conversions
  pcl_ros
  rclcpp
  sensor_msgs
  shape_msgs
  std_msgs
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

# Find required packages
foreach(package ${REQUIRED_PACKAGES})
  find_package(${package} REQUIRED)
endforeach()

# Find PCL components
# Note: 'io' is intentionally omitted. PCL's IO component transitively links the
# static /usr/local/lib/libusb-1.0.a (not built with -fPIC), which breaks the
# final shared-object link. We don't need PCL IO for this package — debug PCD
# saving was removed in favour of publishing clouds as ROS topics.
find_package(PCL REQUIRED COMPONENTS common features filters kdtree search sample_consensus segmentation)

# The upstream mycobot_mtc_pick_place_demo package installs its headers under
# share/<pkg>/<pkg>/ rather than under include/, so the include directory
# exported by that package does not actually exist on disk. Compute the real
# header location and add it explicitly so #include "mycobot_mtc_pick_place_demo/*.h"
# resolves for this package's consumers too.
get_filename_component(_mtc_share_dir "${mycobot_mtc_pick_place_demo_DIR}/.." ABSOLUTE)
set(MTC_INCLUDE_DIR "${_mtc_share_dir}")

# The imported targets exported by mycobot_mtc_pick_place_demo also reference
# the non-existent include/ path in their INTERFACE_INCLUDE_DIRECTORIES, which
# CMake rejects at generate time (it validates every imported target, even
# those we don't link). Patch each imported target to point at the actual
# header location so CMake's existence check passes.
get_filename_component(_mtc_root_dir "${mycobot_mtc_pick_place_demo_DIR}/../../.." ABSOLUTE)
set(MTC_LIB_DIR "${_mtc_root_dir}/lib")
link_directories(${MTC_LIB_DIR})

foreach(_mtc_target
    mycobot_mtc_pick_place_demo::cluster_extraction
    mycobot_mtc_pick_place_demo::get_planning_scene_client
    mycobot_mtc_pick_place_demo::normals_curvature_and_rsd_estimation
    mycobot_mtc_pick_place_demo::object_segmentation
    mycobot_mtc_pick_place_demo::plane_segmentation)
  if(TARGET ${_mtc_target})
    set_target_properties(${_mtc_target} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${MTC_INCLUDE_DIR}")
  endif()
endforeach()

# Set dependencies
set(dependencies ${REQUIRED_PACKAGES})

# Include directories
include_directories(include ${PCL_INCLUDE_DIRS} ${MTC_INCLUDE_DIR})

# Add PCL definitions
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE)

# Define PCL libraries
set(PCL_LIBS
  ${PCL_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_KDTREE_LIBRARIES}
  ${PCL_SEARCH_LIBRARIES}
  ${PCL_SAMPLE_CONSENSUS_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
)

# Libraries
## perception_utils: pure perception logic (reusable, no ROS node plumbing)
add_library(perception_utils SHARED src/perception_utils.cpp)
target_include_directories(perception_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(perception_utils ${dependencies})
target_link_libraries(perception_utils
  ${PCL_LIBS}
  # Reuse the segmentation algorithm libraries from mycobot_mtc_pick_place_demo.
  # We link the bare library names (rather than the imported ::-targets) because
  # the imported targets reference a non-existent include/ path in their
  # INTERFACE_INCLUDE_DIRECTORIES, which CMake rejects at configure time.
  # MTC_INCLUDE_DIR above provides the headers; MTC_LIB_DIR (added via
  # link_directories) provides the .so locations to the linker.
  cluster_extraction
  normals_curvature_and_rsd_estimation
  object_segmentation
  plane_segmentation
)

# Executables
## camera_perception: the main perception node
add_executable(camera_perception src/camera_perception_node.cpp)
target_include_directories(camera_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(camera_perception ${dependencies})
target_link_libraries(camera_perception perception_utils ${PCL_LIBS})

## perception_test_node: subscribes to perception output and reports statistics
add_executable(perception_test_node src/perception_test_node.cpp)
ament_target_dependencies(perception_test_node ${dependencies})

## static_camera_clound: subscribes to /camera/depth_registered/points and
## publishes a static PlanningScene (with one cube CollisionObject) onto
## /test_planning_scene so moveit's PlanningSceneMonitor can apply it.
add_executable(static_camera_clound src/static_camera_clound.cpp)
ament_target_dependencies(static_camera_clound ${dependencies})

# Installation
set(EXECUTABLES
  camera_perception
  perception_test_node
  static_camera_clound
)

set(LIBRARIES
  perception_utils
)

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install headers to include/ (NOT share/) so that the exported include
# directory is valid for downstream packages that consume this package.
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export and package
ament_export_include_directories(include)
ament_export_libraries(${LIBRARIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
