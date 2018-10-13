cmake_minimum_required(VERSION 3.0)
project(RobokasiV2_Host)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SDL 2
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/modules)

if(WIN32)
    set(SDL2_PATH $ENV{SDL2_DIR})
    if(NOT SDL2_PATH)
        message(FATAL_ERROR "Please set SDL2_DIR environment variable to point to SDL 2 installation directory.")
    endif()
endif()

find_package(SDL2 REQUIRED)

# Find OpenGL
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIR)
    set(EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}")
    if(NOT EIGEN3_INCLUDE_DIR)
        message(FATAL_ERROR "Please set EIGEN3_INCLUDE_DIR environment variable to point to SDL 2 installation directory.")
    endif()
endif()


# Add subdirectories
add_subdirectory(ext)
add_subdirectory(include)
add_subdirectory(src)

# Kinematics library target
add_library(kinematics SHARED ${KINEMATICS_SOURCES})
target_include_directories(
        kinematics
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${EIGEN3_INCLUDE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/kinematics
        ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics
)

# GUI library target
add_library(gui SHARED ${GUI_SOURCES})
target_include_directories(
        gui
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${SDL2_INCLUDE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/gui
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
)
target_link_libraries(
        gui
    PUBLIC
        kinematics
)

# Hardware I/O library target
message(${HWIO_SOURCES})
add_library(hwio SHARED ${HWIO_SOURCES})
target_include_directories(
        hwio
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/hwio
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hwio
)

# Robokasi app target
add_executable(Robokasi ${APP_SOURCES})
target_link_libraries(
        Robokasi
    PUBLIC
        gui
        hwio
        ${SDL2_LIBRARY}
        ${OPENGL_LIBRARIES}
        gl3w
        imgui
)
