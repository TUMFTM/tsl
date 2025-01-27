cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tsl_ros2_publisher_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tsl_msgs REQUIRED)
find_package(tsl_logger_cpp REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/tsl_publisher.cpp
  src/msg_creator.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_compile_options(${PROJECT_NAME} PUBLIC -Ofast)


ament_target_dependencies(
  ${PROJECT_NAME} 
  rclcpp
  tsl_msgs
  tsl_logger_cpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(rclcpp tsl_msgs tsl_logger_cpp) # other dependencies so client does not need to call find_package on them

ament_package()

# ==============================================================================
# BUILD EXAMPLES 
# ==============================================================================

set(USAGE_EXAMPLES 
  benchmark
  node_integration
  compression
)

foreach(exec_name ${USAGE_EXAMPLES})
  # Create the executable
  add_executable(${exec_name} examples/${exec_name}.cpp)

  # Link against the lib
  target_link_libraries(${exec_name} ${PROJECT_NAME})
  # Install target
  install(
    TARGETS ${exec_name}  
    DESTINATION examples/${PROJECT_NAME}
  )
endforeach()
