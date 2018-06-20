cmake_minimum_required(VERSION 2.8.3)
project(traxxs_orca)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# find_package(orca REQUIRED)

find_package(catkin REQUIRED COMPONENTS
            roscpp
            orca_ros
            traxxs
            )


catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES traxxs_orca
)

include_directories(
    include/
    ${catkin_INCLUDE_DIRS}
)


add_executable(traxxs_traj src/traxxs_traj.cc)
target_link_libraries(traxxs_traj ${catkin_LIBRARIES})# orca::orca)


add_executable(traxxs_cart_traj src/traxxs_cart_traj.cc)
target_link_libraries(traxxs_cart_traj ${catkin_LIBRARIES})# orca::orca)



install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(TARGETS traxxs_traj
                traxxs_cart_traj
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
