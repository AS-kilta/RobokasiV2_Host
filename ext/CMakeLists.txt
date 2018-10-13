execute_process(COMMAND git submodule update --init)

add_library(
        gl3w
    STATIC
        imgui/examples/libs/gl3w/GL/gl3w.c
)

target_include_directories(
        gl3w
    PUBLIC
        imgui/examples/libs/gl3w
)

target_link_libraries(
        gl3w
    PUBLIC
        ${CMAKE_DL_LIBS}
)

add_library(
        imgui
    STATIC
        imgui/imgui.cpp
        imgui/imgui_draw.cpp
        imgui/imgui_widgets.cpp
        imgui/examples/imgui_impl_opengl3.cpp
        imgui/examples/imgui_impl_sdl.cpp
)

target_include_directories(
        imgui
    PUBLIC
        imgui
        imgui/examples
        ${SDL2_INCLUDE_DIR}
)

target_link_libraries(
        imgui
    PRIVATE
        ${OPENGL_LIBRARIES}
        gl3w
        ${SDL2_LIBRARY}
)

