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
find_package(osqp REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} 
    CATKIN_DEPENDS roscpp visualization_msgs std_msgs nav_msgs tf
)

set(osqp_INCLUDE_DIRS /usr/local/include/osqp)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${osqp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/plan/plan.cpp
    src/global_planner/astar.cpp 
    src/global_planner/hybrid_astar.cpp
    src/global_planner/rrt.cpp
    src/global_planner/rrt_star.cpp
    src/global_planner/jps.cpp
    src/global_planner/lpa_star.cpp
    src/optim/qp_optim.cpp
    src/optim/bezier_optim.cpp
    src/optim/b_spline_optim.cpp
    src/local_planner/local_planner_interface.cpp
    src/local_planner/online_local_planner.cpp 
    src/local_planner/offline_local_planner.cpp
    src/m_util.cpp
    src/m_prm.cpp
)

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(plan src/plan/main.cpp)
add_executable(prm src/m_prm.cpp)

add_dependencies(plan
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)
add_dependencies(prm
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(plan
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    osqp
    OsqpEigen
    ompl
)
target_link_libraries(prm
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    osqp
    OsqpEigen
    ompl
)

install(TARGETS plan
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS prm
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
