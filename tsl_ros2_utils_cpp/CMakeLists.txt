cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tsl_ros2_utils_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tsl_msgs REQUIRED)
find_package(tsl_logger_cpp REQUIRED)
find_package(tsl_ros2_publisher_cpp REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/signal_filter.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)


ament_target_dependencies(
  ${PROJECT_NAME} 
  rclcpp
  tsl_msgs
  tsl_logger_cpp
  tsl_ros2_publisher_cpp
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

# ==============================================================================
# BUILD COMPONENTS 
# ==============================================================================


set(COMPONENT_PATHS components/signal_filter.cpp)
set(COMPONENT_NAMES ${PROJECT_NAME}::SignalFilter)
set(EXECUTABLE_NAMES signal_filter_node)


add_library(${PROJECT_NAME}_components SHARED ${COMPONENT_PATHS})
# Find pkg the additionally required pkgs
find_package(rclcpp_components)
# Link the additional dependencies
ament_target_dependencies(${PROJECT_NAME}_components rclcpp_components)
# Link against the Node lib
target_link_libraries(${PROJECT_NAME}_components ${PROJECT_NAME})

foreach(COMPONENT_NAME EXECUTABLE_NAME IN ZIP_LISTS COMPONENT_NAMES EXECUTABLE_NAMES)
  # Register component and executable for each node
  rclcpp_components_register_node(
    ${PROJECT_NAME}_components
      PLUGIN ${COMPONENT_NAME}
      EXECUTABLE ${EXECUTABLE_NAME}
  )
endforeach()

ament_export_targets(export_${PROJECT_NAME}_components)

install(TARGETS ${PROJECT_NAME}_components
        EXPORT export_${PROJECT_NAME}_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# ==============================================================================
# GENERAL 
# ==============================================================================
ament_export_dependencies(rclcpp tsl_msgs tsl_logger_cpp tsl_ros2_publisher_cpp) # other dependencies so client does not need to call find_package on them

ament_package()
