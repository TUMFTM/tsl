cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tsl_logger_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME}
    INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} INTERFACE c_std_99 cxx_std_17)

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
ament_export_dependencies() # other dependencies so client does not need to call find_package on them
ament_package()


# ==============================================================================
# BUILD EXAMPLES 
# ==============================================================================

set(USAGE_EXAMPLES 
    composition
    log_custom_types
    reference_logger
    value_logger
    benchmark
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
