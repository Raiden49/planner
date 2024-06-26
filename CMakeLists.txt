cmake_minimum_required(VERSION 3.0.2)
project(planner)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    std_msgs
    visualization_msgs
    tf
)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} 
    CATKIN_DEPENDS roscpp visualization_msgs std_msgs nav_msgs tf
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/plan/astar.cpp 
    src/plan/plan.cpp
    src/optim/qp_optim.cpp
    src/optim/bezier_optim.cpp
    src/optim/b_spline_optim.cpp
    src/local_planner/local_planner_interface.cpp
    src/local_planner/online_local_planner.cpp 
    src/local_planner/offline_local_planner.cpp
    src/m_util.cpp
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(plan src/plan/main.cpp)

add_dependencies(plan
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(plan
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    OsqpEigen
)

install(TARGETS plan
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
