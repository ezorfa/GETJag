cmake_minimum_required(VERSION 2.8.3)
project(arm_planning)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  handling_msgs
  sensor_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  geometry_msgs
  actionlib
  operator_interface
)

find_package (Qt5 COMPONENTS Widgets REQUIRED)
find_package (yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs
      INCLUDE_DIRS include

  )

include_directories( include ${catkin_INCLUDE_DIRS})

add_executable (JointStatesPublisher src/JointStatesPublisher.cpp
				     src/ArmControlParameterList.cpp
				     )
add_executable (ComputeFK src/ComputeFK.cpp)
add_executable(init src/init.cpp
                    src/Planning.cpp
                    src/Executor.cpp
                    src/JoyControl.cpp
                    src/GoHomeAction.cpp
                    src/PoseGoalAction.cpp
                    src/ArmControlParameterList.cpp
		    src/GrabAction.cpp
	            src/DropAction.cpp
	            src/InspectAction.cpp
                    src/JointStateSubscriber.cpp
                    src/OpenCloseGripper.cpp
                    src/ExtractAction.cpp
                    )

target_link_libraries(JointStatesPublisher ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(init ${catkin_LIBRARIES} yaml-cpp)
target_link_libraries(ComputeFK ${catkin_LIBRARIES})

add_widget (
    NAME          RobotArmControlWidget
    INCLUDE_DIRS  ${catkin_INCLUDE_DIRS}
    LIBS          ${catkin_LIBRARIES} Qt5::Widgets yaml-cpp
    FILES        widgets/RobotArmControlWidget.cpp
)



