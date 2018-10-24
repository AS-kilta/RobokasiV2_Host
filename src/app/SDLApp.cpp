//
// Created by Lehdari on 2018-10-10.
//

#include <GL/gl3w.h>

#include <imgui.h>
#include <imgui_impl_sdl.h>
#include <imgui_impl_opengl3.h>
#include <cmath>

#include "SDLApp.hpp"


using namespace gui;


SDLApp::SDLApp(const SDLApp::Settings &settings) :
    _settings   (settings),
    _window     (nullptr),
    _quit       (false),
    _renderer   (_shader, _camera)
{
    int err;

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("Error: Could not initialize SDL!\n");
        return;
    }

    _window = SDL_CreateWindow(
         _settings.windowName.c_str(),
         SDL_WINDOWPOS_UNDEFINED,
         SDL_WINDOWPOS_UNDEFINED,
         (int)_settings.windowWidth,
         (int)_settings.windowHeight,
         SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);

    if (_window == nullptr) {
        printf("Error: SDL Window could not be created! SDL_Error: %s\n", SDL_GetError());
        return;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, _settings.glContextMajor);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, _settings.glContextMinor);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, _settings.glContextFlags);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, _settings.glProfileMask);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, _settings.glDoubleBuffer);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);

    _glCtx = SDL_GL_CreateContext(_window);

    if (_glCtx == nullptr) {
        printf("Error: SDL OpenGL context could not be created! SDL_Error: %s\n",
               SDL_GetError());
        return;
    }

    err = gl3wInit();
    if (err) {
        printf("Error: gl3wInit failed\n");
        return;
    }

    ImGui::CreateContext();
    ImGuiIO &imgui_io = ImGui::GetIO();
    ImGui_ImplSDL2_InitForOpenGL(_window, _glCtx);
    ImGui_ImplOpenGL3_Init("#version 330");

    glClearColor(0.f, 0.f, 0.f, 1.f);
    glEnable(GL_DEPTH_TEST);

    _shader.load(std::string(RES_PATH) + "shaders/VS_Simple.glsl",
                 std::string(RES_PATH) + "shaders/FS_Simple.glsl");
    _shader.addUniform("modelToWorld");
    _shader.addUniform("normalToWorld");
    _shader.addUniform("worldToClip");
    _shader.addUniform("Color");
    _camera.lookAt(
        Vec3GLf(0.0f, 20.0f, 40.0f),
        Vec3GLf(0.0f, 0.0f, 0.0f),
        Vec3GLf(0.0f, 1.0f, 0.0f));
    _camera.projection(
        60.0f * M_PI / 180.f,
        (float)_settings.windowWidth / (float)_settings.windowHeight,
        1.0f,
        500.0f);

    _model.loadLinkMeshes();
    _renderables.push_back(&_model);
}

SDLApp::~SDLApp()
{
    // Destroy window and quit SDL subsystems
    SDL_GL_DeleteContext(_glCtx);
    SDL_DestroyWindow(_window);
    SDL_Quit();
}

void SDLApp::loop(void)
{
    // Application main loop
    while (!_quit) {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            handleEvents(event);
        }

        render();
    }
}


void SDLApp::handleEvents(SDL_Event& event)
{
    // Handle SDL events
    switch (event.type) {
        case SDL_QUIT:
            _quit = true;
            break;

        case SDL_KEYDOWN:
            switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    _quit = true;
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

void SDLApp::render(void)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(_window);
    ImGui::NewFrame();

    ImGui::Begin("Window");

    ImGui::End();

    ImGui::Render();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    _renderer.render(_renderables);

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(_window);
}
