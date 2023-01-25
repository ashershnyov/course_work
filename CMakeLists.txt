cmake_minimum_required(VERSION 3.16)
project(mass-spring)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
include(libigl)

# Enable the target igl::glfw
igl_include(glfw)

# Add your project files
add_library(spring spring.h spring.cpp)
target_link_libraries(spring PUBLIC igl::glfw)

add_library(cloth cloth.h cloth.cpp)
target_link_libraries(cloth PUBLIC igl::glfw)

add_library(sphere sphere.h sphere.cpp)
target_link_libraries(sphere PUBLIC igl::glfw)

add_executable(mass-spring main.cpp)
target_link_libraries(mass-spring PUBLIC igl::glfw spring cloth sphere)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
	set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/install" CACHE PATH "..." FORCE)
endif()

set(EXECUTABLE_OUTPUT PATH ${CMAKE_BINARY_DIR})

install(TARGETS mass-spring DESTINATION .)
