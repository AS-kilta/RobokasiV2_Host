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

# Find libserialport viad pkg-config
find_package(PkgConfig)
pkg_check_modules(LIBSERIALPORT libserialport)

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

# Add absolute path to resource directory
# NOTE: Potential portable build should copy resources to the binary folder and use ./ here
add_definitions(-DRES_PATH="${CMAKE_CURRENT_LIST_DIR}/res/")

# Kinematics library target
add_library(kinematics SHARED ${KINEMATICS_SOURCES})
target_compile_options(kinematics
    PRIVATE
    -Wall
    -Wno-c99-extensions
    -pedantic
)
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

target_link_libraries(
        kinematics
    PUBLIC
        nlohmann-json
)

# GUI library target
add_library(gui SHARED ${GUI_SOURCES})
target_compile_options(gui
    PRIVATE
    -Wall
    -Wno-c99-extensions
    -pedantic
)
target_include_directories(
        gui
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${SDL2_INCLUDE_DIR}
        ${LIBSERIALPORT_INCLUDE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/gui
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gui
)
target_link_libraries(
        gui
    PUBLIC
        kinematics
        ${OPENGL_LIBRARIES}
        gl3w
        imgui
)

# Hardware I/O library target
message(${HWIO_SOURCES})
add_library(hwio SHARED ${HWIO_SOURCES})
target_compile_options(hwio
    PRIVATE
    -Wall
    -Wno-c99-extensions
    -pedantic
)
target_include_directories(
        hwio
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${LIBSERIALPORT_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/hwio
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hwio
)

target_link_libraries(
        hwio
    PUBLIC
        nlohmann-json
)

# Robokasi app target
add_executable(Robokasi ${APP_SOURCES})
target_compile_options(Robokasi
    PRIVATE
    -Wall
    -Wno-c99-extensions
    -pedantic
)
target_link_libraries(
        Robokasi
    PUBLIC
        gui
        hwio
        ${SDL2_LIBRARY}
        ${OPENGL_LIBRARIES}
        gl3w
        imgui
        ${LIBSERIALPORT_LIBRARIES}
)

if (NOT LIBSERIALPORT_FOUND)
    target_compile_definitions(kinematics PRIVATE WITHOUT_LIBSERIALPORT)
    target_compile_definitions(gui PRIVATE WITHOUT_LIBSERIALPORT)
    target_compile_definitions(hwio PRIVATE WITHOUT_LIBSERIALPORT)
    target_compile_definitions(Robokasi PRIVATE WITHOUT_LIBSERIALPORT)
endif()
