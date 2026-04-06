cmake_minimum_required(VERSION 3.16)
project(oaslam VERSION 0.1.0 LANGUAGES CXX C)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(BUILD_ORBSLAM3 "Build ORB-SLAM3 adapter" ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "BUILD_ORBSLAM3: ${BUILD_ORBSLAM3}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -faligned-new -Wall -Wno-deprecated-declarations -O3 -march=native")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV not found.")
else()
  if(NOT "${OpenCV_VERSION_MAJOR}" STREQUAL "4")
    message(WARNING "OpenCV 4 is required for dnn module. Object detector will not be available.")
  else()
    message(STATUS "OpenCV 4 found. Enabling DNN-backed observation sources.")
    add_definitions(-DUSE_DNN)
  endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(dlib REQUIRED)

set(LEGACY_INTERNAL_DIR ${PROJECT_SOURCE_DIR}/src/adapters/orbslam2/internal)
set(LEGACY_INTERNAL_INCLUDE_DIR ${LEGACY_INTERNAL_DIR}/include)
set(LEGACY_INTERNAL_SOURCE_DIR ${LEGACY_INTERNAL_DIR}/src)

set(LEGACY_INTERNAL_SOURCES
  ${LEGACY_INTERNAL_SOURCE_DIR}/System.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Tracking.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/LocalMapping.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/LoopClosing.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/ORBextractor.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/ORBmatcher.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/FrameDrawer.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Converter.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/MapPoint.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/KeyFrame.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Map.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/MapDrawer.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Optimizer.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/PnPsolver.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Frame.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/KeyFrameDatabase.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Sim3Solver.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Initializer.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Viewer.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/ImageDetections.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Ellipse.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Ellipsoid.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/ObjectTrack.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Distance.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/MapObject.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/OptimizerObject.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/LocalObjectMapping.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Utils.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/ColorManager.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Reconstruction.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Camera.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Scene.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/p3p.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/Localization.cc
  ${LEGACY_INTERNAL_SOURCE_DIR}/ARViewer.cc
)

set(OASLAM_COMMON_SOURCES
  src/common/file_utils.cc
  src/common/opencv_utils.cc
  src/common/eigen_utils.cc
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(oaslam_core INTERFACE)
target_include_directories(oaslam_core INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(oaslam_core INTERFACE ${OpenCV_LIBS})

add_library(oaslam_ports INTERFACE)
target_link_libraries(oaslam_ports INTERFACE oaslam_core)

add_library(oaslam_common STATIC ${OASLAM_COMMON_SOURCES})
target_include_directories(oaslam_common
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(oaslam_common PUBLIC oaslam_ports Eigen3::Eigen ${OpenCV_LIBS} stdc++fs)

add_library(oaslam_legacy_internal STATIC ${LEGACY_INTERNAL_SOURCES})
target_include_directories(oaslam_legacy_internal PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${LEGACY_INTERNAL_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(oaslam_legacy_internal PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
  stdc++fs
  dlib::dlib
)

add_library(oaslam_adapter_observations STATIC
  src/adapters/observations/file_observation_source.cc
  src/adapters/observations/onnx_observation_source.cc
)
target_include_directories(oaslam_adapter_observations
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(oaslam_adapter_observations PUBLIC
  oaslam_ports
  oaslam_common
  oaslam_legacy_internal
)

add_library(oaslam_adapter_orbslam2 STATIC
  src/adapters/orbslam2/orbslam2_backend_adapter.cc
)
target_include_directories(oaslam_adapter_orbslam2
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${LEGACY_INTERNAL_INCLUDE_DIR}
)
target_link_libraries(oaslam_adapter_orbslam2 PUBLIC
  oaslam_ports
  oaslam_common
  oaslam_legacy_internal
)

# ===========================================================================
# ORB-SLAM3 adapter (conditional)
# ===========================================================================
if(BUILD_ORBSLAM3)

set(ORBSLAM3_INTERNAL_DIR ${PROJECT_SOURCE_DIR}/src/adapters/orbslam3/internal)
set(ORBSLAM3_INTERNAL_INCLUDE_DIR ${ORBSLAM3_INTERNAL_DIR}/include)
set(ORBSLAM3_INTERNAL_SOURCE_DIR ${ORBSLAM3_INTERNAL_DIR}/src)
set(ORBSLAM3_THIRDPARTY_DIR ${ORBSLAM3_INTERNAL_DIR}/Thirdparty)

# --- Sophus (header-only, bundled in Thirdparty) ---
set(ORBSLAM3_SOPHUS_INCLUDE_DIR ${ORBSLAM3_THIRDPARTY_DIR}/Sophus)
if(NOT EXISTS "${ORBSLAM3_SOPHUS_INCLUDE_DIR}/sophus/se3.hpp")
  # Fall back to system-installed Sophus
  find_package(Sophus REQUIRED)
  set(ORBSLAM3_SOPHUS_INCLUDE_DIR ${Sophus_INCLUDE_DIRS})
  message(STATUS "ORB-SLAM3: Using system Sophus at ${ORBSLAM3_SOPHUS_INCLUDE_DIR}")
else()
  message(STATUS "ORB-SLAM3: Using bundled Sophus at ${ORBSLAM3_SOPHUS_INCLUDE_DIR}")
endif()

# --- g2o config.h generation for ORB-SLAM3 ---
set(ORBSLAM3_G2O_DIR ${ORBSLAM3_THIRDPARTY_DIR}/g2o)
set(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(
  ${ORBSLAM3_G2O_DIR}/config.h.in
  ${ORBSLAM3_G2O_DIR}/config.h
)

# --- ORB-SLAM3 Thirdparty g2o (static, separate target name to avoid conflict) ---
add_library(orbslam3_g2o STATIC
  ${ORBSLAM3_G2O_DIR}/g2o/types/types_sba.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/types/types_six_dof_expmap.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/types/types_seven_dof_expmap.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/types/se3mat.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/hyper_graph_action.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/hyper_graph.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/marginal_covariance_cholesky.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/matrix_structure.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/batch_stats.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/optimizable_graph.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/solver.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/optimization_algorithm_factory.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/estimate_propagator.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/factory.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/sparse_optimizer.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/hyper_dijkstra.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/parameter_container.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/parameter.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/optimization_algorithm.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/optimization_algorithm_with_hessian.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/optimization_algorithm_levenberg.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/optimization_algorithm_gauss_newton.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/optimization_algorithm_dogleg.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/jacobian_workspace.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/robust_kernel.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/robust_kernel_factory.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/robust_kernel_impl.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/core/cache.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/stuff/timeutil.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/stuff/os_specific.c
  ${ORBSLAM3_G2O_DIR}/g2o/stuff/string_tools.cpp
  ${ORBSLAM3_G2O_DIR}/g2o/stuff/property.cpp
)
target_include_directories(orbslam3_g2o PUBLIC
  ${ORBSLAM3_G2O_DIR}
  ${ORBSLAM3_G2O_DIR}/g2o/core
  ${ORBSLAM3_G2O_DIR}/g2o/types
  ${ORBSLAM3_G2O_DIR}/g2o/stuff
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(orbslam3_g2o PUBLIC Eigen3::Eigen)
set_target_properties(orbslam3_g2o PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
)

# --- ORB-SLAM3 Thirdparty DBoW2 (static, separate target name) ---
set(ORBSLAM3_DBOW2_DIR ${ORBSLAM3_THIRDPARTY_DIR}/DBoW2)
add_library(orbslam3_DBoW2 STATIC
  ${ORBSLAM3_DBOW2_DIR}/DBoW2/BowVector.cpp
  ${ORBSLAM3_DBOW2_DIR}/DBoW2/FORB.cpp
  ${ORBSLAM3_DBOW2_DIR}/DBoW2/FeatureVector.cpp
  ${ORBSLAM3_DBOW2_DIR}/DBoW2/ScoringObject.cpp
  ${ORBSLAM3_DBOW2_DIR}/DUtils/Random.cpp
  ${ORBSLAM3_DBOW2_DIR}/DUtils/Timestamp.cpp
)
target_include_directories(orbslam3_DBoW2 PUBLIC
  ${ORBSLAM3_DBOW2_DIR}
)
target_link_libraries(orbslam3_DBoW2 PUBLIC ${OpenCV_LIBS})
set_target_properties(orbslam3_DBoW2 PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
)

# --- ORB-SLAM3 internal library ---
set(ORBSLAM3_INTERNAL_SOURCES
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/System.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Tracking.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/LocalMapping.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/LoopClosing.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/ORBextractor.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/ORBmatcher.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/FrameDrawer.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Converter.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/MapPoint.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/KeyFrame.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Map.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Atlas.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/MapDrawer.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Optimizer.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Frame.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/KeyFrameDatabase.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Sim3Solver.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Viewer.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/ImageDetections.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Ellipse.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Ellipsoid.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/ObjectTrack.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Distance.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/MapObject.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/OptimizerObject.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/LocalObjectMapping.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Utils.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/ColorManager.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Reconstruction.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Camera.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/p3p.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Localization.cc
  # ORB-SLAM3 specific files
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Config.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/Settings.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/G2oTypes.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/GeometricTools.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/ImuTypes.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/TwoViewReconstruction.cc
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/OptimizableTypes.cpp
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/MLPnPsolver.cpp
  # Camera models
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/CameraModels/KannalaBrandt8.cpp
  ${ORBSLAM3_INTERNAL_SOURCE_DIR}/CameraModels/Pinhole.cpp
)

add_library(oaslam_orbslam3_internal STATIC ${ORBSLAM3_INTERNAL_SOURCES})
target_include_directories(oaslam_orbslam3_internal
  PUBLIC
    ${ORBSLAM3_INTERNAL_INCLUDE_DIR}
    ${ORBSLAM3_INTERNAL_INCLUDE_DIR}/CameraModels
  PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${ORBSLAM3_INTERNAL_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${ORBSLAM3_SOPHUS_INCLUDE_DIR}
    ${ORBSLAM3_G2O_DIR}
    ${ORBSLAM3_DBOW2_DIR}
)
target_link_libraries(oaslam_orbslam3_internal PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${Pangolin_LIBRARIES}
  orbslam3_g2o
  orbslam3_DBoW2
  stdc++fs
  dlib::dlib
)
set_target_properties(oaslam_orbslam3_internal PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
)

# --- ORB-SLAM3 adapter ---
add_library(oaslam_adapter_orbslam3 STATIC
  src/adapters/orbslam3/orbslam3_backend_adapter.cc
  src/adapters/orbslam3/orbslam3_pose_utils.cc
)
target_include_directories(oaslam_adapter_orbslam3
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${ORBSLAM3_INTERNAL_INCLUDE_DIR}
    ${ORBSLAM3_INTERNAL_INCLUDE_DIR}/CameraModels
    ${ORBSLAM3_SOPHUS_INCLUDE_DIR}
    ${ORBSLAM3_G2O_DIR}
)
target_link_libraries(oaslam_adapter_orbslam3 PUBLIC
  oaslam_ports
  oaslam_common
  oaslam_orbslam3_internal
)
set_target_properties(oaslam_adapter_orbslam3 PROPERTIES
  CXX_STANDARD 17
)

# Compile definition so module_factories.cc knows ORB-SLAM3 is available
add_definitions(-DBUILD_ORBSLAM3)

endif(BUILD_ORBSLAM3)
# ===========================================================================

add_library(oaslam_adapter_oa_objects STATIC
  src/adapters/oa_objects/oa_object_pipeline_adapter.cc
)
target_include_directories(oaslam_adapter_oa_objects
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(oaslam_adapter_oa_objects PUBLIC
  oaslam_ports
)

add_library(oaslam_adapter_visualization STATIC
  src/adapters/visualization/pangolin_visualizer.cc
)
target_include_directories(oaslam_adapter_visualization
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(oaslam_adapter_visualization PUBLIC
  oaslam_ports
)

add_library(oaslam_adapter_agent STATIC
  src/adapters/agent/null_agent_gateway.cc
)
target_include_directories(oaslam_adapter_agent
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(oaslam_adapter_agent PUBLIC
  oaslam_ports
)

add_library(oaslam_app STATIC
  src/app/slam_session.cc
  src/app/module_factories.cc
)
target_include_directories(oaslam_app
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(oaslam_app PUBLIC
  oaslam_ports
  oaslam_common
  oaslam_adapter_orbslam2
  oaslam_adapter_observations
  oaslam_adapter_oa_objects
  oaslam_adapter_visualization
  oaslam_adapter_agent
)
if(BUILD_ORBSLAM3)
  target_link_libraries(oaslam_app PUBLIC oaslam_adapter_orbslam3)
endif()

add_library(oaslam_sdk INTERFACE)
target_include_directories(oaslam_sdk INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(oaslam_sdk INTERFACE
  oaslam_core
  oaslam_ports
  oaslam_app
)

add_executable(oa-slam examples/oa-slam.cc)
target_link_libraries(oa-slam PRIVATE oaslam_sdk)

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
set(_OASLAM_INSTALL_TARGETS
  oaslam_core
  oaslam_ports
  oaslam_common
  oaslam_legacy_internal
  oaslam_adapter_observations
  oaslam_adapter_orbslam2
  oaslam_adapter_oa_objects
  oaslam_adapter_visualization
  oaslam_adapter_agent
  oaslam_app
  oaslam_sdk
  oa-slam
)
if(BUILD_ORBSLAM3)
  list(APPEND _OASLAM_INSTALL_TARGETS
    orbslam3_g2o
    orbslam3_DBoW2
    oaslam_orbslam3_internal
    oaslam_adapter_orbslam3
  )
endif()

install(TARGETS ${_OASLAM_INSTALL_TARGETS}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/oaslam DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Generate a lightweight package config so that the ROS 2 wrapper (and any
# other downstream project) can do  find_package(oaslam CONFIG REQUIRED)
# and link against oaslam::oaslam_sdk.
# We deliberately avoid install(EXPORT …) because the many internal static
# libraries carry source-tree absolute paths in their INTERFACE properties
# which CMake's export validation rejects.

# Write the template with @PACKAGE_INIT@ for configure_package_config_file
# and hard-coded relative paths for lib/include dirs.
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfig.cmake.in
[=[
@PACKAGE_INIT@

include(CMakeFindDependencyMacro)
find_dependency(OpenCV REQUIRED)
find_dependency(Eigen3 REQUIRED)
find_dependency(Pangolin REQUIRED)
find_dependency(dlib REQUIRED)

# Create an IMPORTED INTERFACE target that downstream can link against.
if(NOT TARGET oaslam::oaslam_sdk)
  add_library(oaslam::oaslam_sdk INTERFACE IMPORTED)

  # Collect every static/shared library we installed
  file(GLOB _oaslam_libs "${CMAKE_CURRENT_LIST_DIR}/../../lib*.a")
  file(GLOB _oaslam_shared "${CMAKE_CURRENT_LIST_DIR}/../../lib*.so")
  list(APPEND _oaslam_all_libs ${_oaslam_libs} ${_oaslam_shared})

  get_filename_component(_oaslam_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)

  set_target_properties(oaslam::oaslam_sdk PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_oaslam_prefix}/include"
    INTERFACE_LINK_LIBRARIES "${_oaslam_all_libs};Eigen3::Eigen;${OpenCV_LIBS};${Pangolin_LIBRARIES};dlib::dlib;stdc++fs"
  )
  unset(_oaslam_libs)
  unset(_oaslam_shared)
  unset(_oaslam_all_libs)
  unset(_oaslam_prefix)
endif()
]=]
)

configure_package_config_file(
  ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/oaslam
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfig.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/oaslam
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/oaslam
)
