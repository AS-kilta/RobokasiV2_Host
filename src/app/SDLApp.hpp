//
// Created by Lehdari on 2018-10-10.
//

#ifndef SDL_APP_HPP
#define SDL_APP_HPP


#include <SDL.h>
#include <string>

#include "gui/Camera.hpp"
#include "gui/Shader.hpp"
#include "gui/Puma560Model.hpp"
#include "gui/Renderer.hpp"


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
            int64_t width = 800,
            int64_t height = 600,
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

    struct Settings {
        WindowSettings window;
        GLSettings     gl;

        explicit Settings(
            const WindowSettings& window = WindowSettings(),
            const GLSettings& gl = GLSettings()) :
            window(window),
            gl(gl)
        {}
    };

    explicit SDLApp(const Settings& settings = Settings());
    ~SDLApp();

    void loop(void);

private:
    Settings        _settings;
    SDL_Window*     _window;
    SDL_GLContext   _glCtx;
    bool            _quit; // flag for quitting the application

    gui::Shader     _shader;
    gui::Camera     _camera;
    gui::Renderer   _renderer;

    gui::Puma560Model               _model;
    std::vector<gui::Renderable*>   _renderables;

    // Window event handling loop
    void handleEvents(SDL_Event& event);
    void render(void);
};


#endif // SDL_APP_HPPs
