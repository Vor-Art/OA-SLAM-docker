cmake_minimum_required(VERSION 3.16)
project(oaslam VERSION 0.1.0 LANGUAGES CXX)

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

install(TARGETS
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
  EXPORT oaslamTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/oaslam DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

configure_package_config_file(
  ${PROJECT_SOURCE_DIR}/cmake/oaslamConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/oaslam
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/oaslamConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/oaslam
)

install(
  EXPORT oaslamTargets
  FILE oaslamTargets.cmake
  NAMESPACE oaslam::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/oaslam
)
