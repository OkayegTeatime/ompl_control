cmake_minimum_required(VERSION 3.8)
project(ompl_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tutorial_interfaces REQUIRED)

include_directories(
    include
    ${OMPL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

# add_executable(control_planning src/control_planning.cpp)
# add_executable(ompl2ros src/ompl2ros.cpp)


file(GLOB DEMO_SOURCES RELATIVE ${CMAKE_SOURCE_DIR}/src/ src/*.cpp )
foreach( demoFile ${DEMO_SOURCES} )
    string( REPLACE ".cpp" "" demoName ${demoFile} )
    add_executable( ${demoName} src/${demoFile} )
    # Make sure YourLib is linked to each app
    target_link_libraries( ${demoName} ${OMPL_LIBRARIES} ${YAML_CPP_LIBRARIES} ode)
    install(TARGETS ${demoName} DESTINATION lib/${PROJECT_NAME})
endforeach( demoFile ${DEMO_SOURCES} )

ament_target_dependencies(control_planning
    rclcpp
    ompl
    yaml-cpp
    tutorial_interfaces
)

ament_target_dependencies(ompl2ros
    rclcpp
    nav_msgs
    ackermann_msgs
    tutorial_interfaces
)


install(
    DIRECTORY include launch
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()

