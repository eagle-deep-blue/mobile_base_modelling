cmake_minimum_required(VERSION 2.8.3)
project(autonomous_locomotor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge	
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(Eigen3)
find_package(OpenCV)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autonomous_locomotor
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
set(KINEMATICS kinematics_mobile_base)
set(PLANNER planner)
set(CONTROLLER controller)

include_directories(                                                                                
# include                                                                                           
  include/${KINEMATICS}
  include/${PLANNER}
  include/${CONTROLLER}
  ${catkin_INCLUDE_DIRS}                                                                            
  ${EIGEN3_INCLUDE_DIR}                                                                             
  ${OpenCV_INCLUDE_DIRS}
)                                                                                                   

#######################################################################                            
set(LIB_DIR ${PROJECT_SOURCE_DIR}/lib)
set(LIBS 
	${LIB_DIR}/libkinematics_mecanum_base.so
	${LIB_DIR}/libmobile_base_controller.so
	${LIB_DIR}/libomni_base_controller.so
	${LIB_DIR}/libunicycle_base_controller.so
	)
message(${LIBS})

set(MOBILE_BASE_DIR src/mobile_base)
add_executable(mecanum_mobile_base_node ${MOBILE_BASE_DIR}/mecanum_mobile_base_node.cpp)
target_link_libraries(mecanum_mobile_base_node
   ${catkin_LIBRARIES}                                                                              
   ${LIBS}
   #   kinematics_mecanum_base 
 )                                                                                                  
add_executable(fake_wheel_encoder_node ${MOBILE_BASE_DIR}/fake_wheel_encoder_node.cpp)          
target_link_libraries(fake_wheel_encoder_node                                                       
    ${catkin_LIBRARIES}                                                                             
)                                                                         

#---------------------------------------------------------------------------------------
set(PERCEPTION_DIR src/perception)
add_executable(perception_lidar_node ${PERCEPTION_DIR}/perception_lidar_node.cpp)               
target_link_libraries(perception_lidar_node                                                     
   ${catkin_LIBRARIES}                                                                              
   ${OpenCV_LIBS}                                                                                   
 )                            
add_executable(perception_rgb_node ${PERCEPTION_DIR}/perception_rgb_node.cpp)                   
target_link_libraries(perception_rgb_node                                                     
   ${catkin_LIBRARIES}                                                                              
   ${OpenCV_LIBS}                                                                                   
 )                            

add_executable(perception_imu_rgb_node ${PERCEPTION_DIR}/perception_imu_rgb_node.cpp)           
target_link_libraries(perception_imu_rgb_node                                                     
   ${catkin_LIBRARIES}                                                                              
   ${OpenCV_LIBS}                                                                                   
 )                     
 
 
 add_executable(test_synchronization ${PERCEPTION_DIR}/test_synchronization.cpp)
 target_link_libraries(test_synchronization ${catkin_LIBRARIES})
                                                                                                    
#---------------------------------------------------------------------------------------
add_executable(velocity_smoother_node src/${CONTROLLER}/velocity_smoother.cpp)                    
target_link_libraries(velocity_smoother_node ${catkin_LIBRARIES})

add_executable(motor_velocity_controller_node  src/test/motor_velocity_controller.cpp)    
target_link_libraries(motor_velocity_controller_node ${catkin_LIBRARIES})                           
                                                                                                    
add_executable(base_velocity_controller_node src/test/base_velocity_controller.cpp)      
target_link_libraries(base_velocity_controller_node ${catkin_LIBRARIES})                            
########################################################################                                                                                                                                
                            
