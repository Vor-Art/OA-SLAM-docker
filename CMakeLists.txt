cmake_minimum_required(VERSION 3.16)
project(oaslam VERSION 0.1.0 LANGUAGES CXX C)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

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

set(OASLAM_COMMON_SOURCES
  src/common/file_utils.cc
  src/common/opencv_utils.cc
  src/common/eigen_utils.cc
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

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

# ===========================================================================
# ORB-SLAM3 adapter
# ===========================================================================

find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

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

# --- g2o for ORB-SLAM3 (config.h is pre-generated in the source tree) ---
set(ORBSLAM3_G2O_DIR ${ORBSLAM3_THIRDPARTY_DIR}/g2o)

# --- ORB-SLAM3 Thirdparty g2o (static) ---
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

# --- ORB-SLAM3 Thirdparty DBoW2 (static) ---
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
  OpenSSL::Crypto
  Boost::serialization
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
    ${ORBSLAM3_INTERNAL_DIR}
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
  oaslam_orbslam3_internal
)

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
  oaslam_adapter_orbslam3
  oaslam_adapter_observations
  oaslam_adapter_oa_objects
  oaslam_adapter_visualization
  oaslam_adapter_agent
)

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

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------
set(_OASLAM_INSTALL_TARGETS
  oaslam_core
  oaslam_ports
  oaslam_common
  orbslam3_g2o
  orbslam3_DBoW2
  oaslam_orbslam3_internal
  oaslam_adapter_orbslam3
  oaslam_adapter_observations
  oaslam_adapter_oa_objects
  oaslam_adapter_visualization
  oaslam_adapter_agent
  oaslam_app
  oaslam_sdk
)

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

# Use the source-tree template (cmake/oaslamConfig.cmake.in) instead of
# file(WRITE) to avoid Ninja infinite re-configure loop.
configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/oaslamConfig.cmake.in
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
