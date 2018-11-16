//
// Created by Lehdari on 2018-10-10.
//

#include <GL/gl3w.h>

#include <imgui.h>
#include <imgui_impl_sdl.h>
#include <imgui_impl_opengl3.h>

#include <cmath>
#include <functional>

#include "SDLApp.hpp"

#include "hwio/SerialProto.hpp"
#include "hwio/CommandQueue.hpp"
#include "gui/SerialConfig.hpp"
#include "gui/DriveControl.hpp"
#include "kinematics/Program.hpp"
#include "gui/ProgramEditor.hpp"

using namespace gui;


SDLApp::SDLApp(const SDLApp::Settings &settings) :
    _settings   (settings),
    _window     (nullptr),
    _quit       (false),
    _lineRenderer(_lineShader, _camera),
    _meshRenderer(_meshShader, _camera),
    _serialProto        (),
    _commandQueue       (_serialProto),
    _serialConfigGui    (_serialProto),
    _driveControlGui    (_serialProto, _commandQueue, _visualizerConfig),
    _programEditor      (_program, _visualizerConfig)
{
    int err;

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("Error: Could not initialize SDL!\n");
        return;
    }

    _window = SDL_CreateWindow(
         _settings.window.name.c_str(),
         SDL_WINDOWPOS_UNDEFINED,
         SDL_WINDOWPOS_UNDEFINED,
         (int)_settings.window.width,
         (int)_settings.window.height,
         SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL);

    if (_window == nullptr) {
        printf("Error: SDL Window could not be created! SDL_Error: %s\n", SDL_GetError());
        return;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, _settings.gl.contextMajor);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, _settings.gl.contextMinor);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, _settings.gl.contextFlags);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, _settings.gl.profileMask);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, _settings.gl.doubleBuffer);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);

    _glCtx = SDL_GL_CreateContext(_window);

    if (_glCtx == nullptr) {
        printf("Error: SDL OpenGL context could not be created! SDL_Error: %s\n",
               SDL_GetError());
        return;
    }

    // Load OpenGL binds
    err = gl3wInit();
    if (err) {
        printf("Error: gl3wInit failed\n");
        return;
    }

    // Initialize imgui
    ImGui::CreateContext();
    ImGuiIO &imgui_io = ImGui::GetIO();
    ImGui_ImplSDL2_InitForOpenGL(_window, _glCtx);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Initialize OpenGL
    glViewport(0, 0, _settings.window.width, _settings.window.height);
    glClearColor(0.2f, 0.2f, 0.2f, 1.f);
    glEnable(GL_DEPTH_TEST);

    // Initialize resources
    _meshShader.load(std::string(RES_PATH) + "shaders/VS_Simple.glsl",
                     std::string(RES_PATH) + "shaders/FS_Simple.glsl");
    _meshShader.addUniform("objectToWorld");
    _meshShader.addUniform("normalToWorld");
    _meshShader.addUniform("worldToClip");
    _meshShader.addUniform("Color");

    _lineShader.load(std::string(RES_PATH) + "shaders/VS_Lines.glsl",
                     std::string(RES_PATH) + "shaders/FS_Lines.glsl");
    _lineShader.addUniform("modelToClip");

    _camera.lookAt(
        Vec3GLf(0.0f, 1500.0f, 2500.0f),
        Vec3GLf(0.0f, 500.0f, 0.0f),
        Vec3GLf(0.0f, 1.0f, 0.0f));
/*
    _camera.lookAt(
        Vec3GLf(0.0f, 2.0f, 4.0f),
        Vec3GLf(0.0f, 0.0f, 0.0f),
        Vec3GLf(0.0f, 1.0f, 0.0f));
*/
    _camera.projection(
        _settings.camera.fov,
        (float)_settings.window.width / (float)_settings.window.height,
        _settings.camera.near,
        _settings.camera.far);

    _model = std::make_shared<Puma560Model>();
    _renderables.push_back(_model);
    _coordinateFrame = Lines::CoordinateFrame();

    _serialProto.registerBufNotifyCb(0.3f, std::bind(&hwio::CommandQueue::bufferNotify,
                                                     &_commandQueue,
                                                     std::placeholders::_1,
                                                     std::placeholders::_2));
}

SDLApp::~SDLApp()
{
    // Destroy imgui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

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
            // Skip events if imgui widgets are being modified
            if (ImGui::IsAnyItemActive())
                return;
            switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                    _quit = true;
                    break;

                default:
                    break;
            }
            break;

        case SDL_MOUSEMOTION:
            // Skip mouse events on imgui windows and widgets
            if (ImGui::IsMouseHoveringAnyWindow() || ImGui::IsAnyItemActive())
                break;
            // Drag with right mouse button controls trackball
            if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(3))
                _trackball.handleMouseMove(event.motion.xrel, event.motion.yrel);
            break;

        case SDL_MOUSEWHEEL:
            // Skip mouse events on imgui windows and widgets
            if (ImGui::IsMouseHoveringAnyWindow() || ImGui::IsAnyItemActive())
                break;
            _trackball.handleMouseScroll(event.wheel.y);
            break;

        case SDL_WINDOWEVENT:
            switch (event.window.event) {
                case SDL_WINDOWEVENT_RESIZED:
                case SDL_WINDOWEVENT_SIZE_CHANGED:
                    _settings.window.width = event.window.data1;
                    _settings.window.height = event.window.data2;
                    glViewport(0, 0, _settings.window.width, _settings.window.height);
                    _camera.projection(
                        _settings.camera.fov,
                        (float)_settings.window.width / (float)_settings.window.height,
                        _settings.camera.near,
                        _settings.camera.far);
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
    // Initialize imgui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(_window);
    ImGui::NewFrame();

    // Generate widgets
    _serialConfigGui.render();
    _driveControlGui.render();
    _visualizerConfig.render(_model);
    _programEditor.render();

    // Generate draw data
    ImGui::Render();

    _trackball.doLookAt(_camera);

    // Render geometry
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (_visualizerConfig.meshRenderEnable)
        _meshRenderer.render(_renderables);
    if (_visualizerConfig.frameRenderEnable)
        _lineRenderer.render(_model->getCoordinateFrames(_coordinateFrame));

    // Render imgui
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    // Swap draw and display buffers
    SDL_GL_SwapWindow(_window);
}
