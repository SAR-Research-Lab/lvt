cmake_minimum_required(VERSION 3.1)
project(lvt)

set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/lvt/cmake/")

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

add_compile_options(-march=native -fvisibility=hidden -fPIC)

find_package(OpenCV 3.1)
list(APPEND INC_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND LINK_LIBS ${OpenCV_LIBS})

find_package(Eigen3)
list(APPEND INC_DIRS ${EIGEN3_INCLUDE_DIR} )

find_package(G2O)
set(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SBA} ${G2O_TYPES_SLAM3D} ${G2O_SOLVER_PCG})
list(APPEND INC_DIRS ${G2O_INCLUDE_DIR})
list(APPEND LINK_LIBS ${G2O_LIBRARIES})

option(ENABLE_VISUALIZATION "Enable visualization. This requires dependecy on Pangolin." ON )
if(ENABLE_VISUALIZATION)
    find_package(Pangolin)
    add_definitions(-DLVT_ENABLE_VISUALIZATION)
    list(APPEND INC_DIRS ${Pangolin_INCLUDE_DIRS})
    list(APPEND LINK_LIBS ${Pangolin_LIBRARIES})
endif()

include_directories( 
    ${CMAKE_SOURCE_DIR}/lvt/src
    ${INC_DIRS} )
    
add_library(lvt
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_pose.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_motion_model.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_image_features_struct.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_image_features_handler.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_local_map.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_pnp_solver.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_parameters.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_system.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_visualization.cpp
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_logging_utils.cpp
    )
    
add_library(lvt_c SHARED
    ${CMAKE_SOURCE_DIR}/lvt/src/lvt_c.cpp
    )

target_link_libraries(lvt_c
    lvt
    -pthread
    ${LINK_LIBS}
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/kitti)
add_executable(kitti_example
    ${CMAKE_SOURCE_DIR}/examples/kitti/kitti_example.cpp
    )

target_link_libraries(kitti_example
    lvt
    -pthread
    ${LINK_LIBS}
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/euroc)
add_executable(euroc_example
    ${CMAKE_SOURCE_DIR}/examples/euroc/euroc_example.cpp
    )

target_link_libraries(euroc_example
    lvt
    -pthread
    ${LINK_LIBS}
    )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/tum_rgbd)
add_executable(tum_rgbd_example
    ${CMAKE_SOURCE_DIR}/examples/tum_rgbd/tum_rgbd_example.cpp
    )

target_link_libraries(tum_rgbd_example
    lvt
    -pthread
    ${LINK_LIBS}
    )
