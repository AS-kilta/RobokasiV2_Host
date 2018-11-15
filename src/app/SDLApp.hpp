//
// Created by Lehdari on 2018-10-10.
//

#ifndef SDL_APP_HPP
#define SDL_APP_HPP


#include <memory>
#include <SDL.h>
#include <string>

#include "gui/Camera.hpp"
#include "gui/Shader.hpp"
#include "gui/Puma560Model.hpp"
#include "gui/Renderer.hpp"
#include "gui/Lines.hpp"
#include "gui/SerialConfig.hpp"
#include "gui/DriveControl.hpp"
#include "gui/Trackball.hpp"
#include "gui/VisualizerConfig.hpp"
#include "gui/ProgramEditor.hpp"
#include "gui/ProgramAnimator.hpp"

#include "hwio/SerialProto.hpp"
#include "hwio/CommandQueue.hpp"

#include "kinematics/Program.hpp"

class SDLApp {
public:
    // Settings for the application
    struct WindowSettings {
        std::string       name;
        int64_t           width;
        int64_t           height;
        int64_t           framerateLimit;

        explicit WindowSettings(
            const std::string& name = "",
            int64_t width = 1280,
            int64_t height = 720,
            int64_t framerateLimit = 60) :
            name(name),
            width(width),
            height(height),
            framerateLimit(framerateLimit)
        {}
    };

    struct GLSettings {
        int32_t           contextMajor;
        int32_t           contextMinor;
        SDL_GLcontextFlag contextFlags;
        SDL_GLprofile     profileMask;
        bool              doubleBuffer;

        explicit GLSettings(
            int32_t contextMajor = 3,
            int32_t contextMinor = 3,
            SDL_GLcontextFlag contextFlags = SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG,
            SDL_GLprofile profileMask = SDL_GL_CONTEXT_PROFILE_CORE,
            bool doubleBuffer = true) :
            contextMajor(contextMajor),
            contextMinor(contextMinor),
            contextFlags(contextFlags),
            profileMask(profileMask),
            doubleBuffer(doubleBuffer)
        {}
    };

    struct CameraSettings {
        gui::Vec3GLf pos;
        gui::Vec3GLf target;
        gui::Vec3GLf up;
        float        fov;
        float        near;
        float        far;

        explicit CameraSettings(
            const gui::Vec3GLf& pos = gui::Vec3GLf(0.f, 20.f, 40.f),
            const gui::Vec3GLf& target = gui::Vec3GLf(0.f, 0.f, 0.f),
            const gui::Vec3GLf& up = gui::Vec3GLf(0.f, 1.f, 0.f),
            float fov = 60.f * PI / 180.f,
            float near = 100.f,
            float far = 10000.f):
            pos(pos),
            target(target),
            up(up),
            fov(fov),
            near(near),
            far(far)
        {}
    };

    struct Settings {
        WindowSettings window;
        GLSettings     gl;
        CameraSettings camera;

        explicit Settings(
            const WindowSettings& window = WindowSettings(),
            const GLSettings& gl = GLSettings(),
            const CameraSettings& camera = CameraSettings()) :
            window(window),
            gl(gl),
            camera(camera)
        {}
    };

    explicit SDLApp(const Settings& settings = Settings());
    ~SDLApp();

    void loop(void);

private:
    Settings            _settings;
    SDL_Window*         _window;
    SDL_GLContext       _glCtx;
    bool                _quit; // flag for quitting the application
    uint32_t            _lastTicks;
    uint32_t            _frameTicks;

    hwio::SerialProto   _serialProto;
    hwio::CommandQueue  _commandQueue;

    gui::VisualizerConfig   _visualizerConfig;

    kin::Program    _program;

    gui::SerialConfig       _serialConfigGui;
    gui::DriveControl       _driveControlGui;
    gui::ProgramEditor      _programEditor;
    gui::ProgramAnimator    _programAnimator;

    gui::Shader     _lineShader;
    gui::Shader     _meshShader;
    gui::Camera     _camera;
    gui::Trackball  _trackball;
    gui::Renderer   _lineRenderer;
    gui::Renderer   _meshRenderer;

    std::shared_ptr<gui::Puma560Model>              _model;
    std::vector<std::shared_ptr<gui::Renderable>>   _renderables;
    std::shared_ptr<gui::Lines>                     _coordinateFrame;

    // Window event handling loop
    void handleEvents(SDL_Event& event);
    void render(void);
};


#endif // SDL_APP_HPPs
