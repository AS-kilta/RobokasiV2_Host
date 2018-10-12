//
// Created by Lehdari on 2018-10-10.
//

#ifndef SDL_APP_HPP
#define SDL_APP_HPP


#include <SDL.h>
#include <string>


class SDLApp {
public:
    // Settings for the application
    struct Settings {
        std::string       windowName;
        int64_t           windowWidth;
        int64_t           windowHeight;
        int64_t           windowFramerateLimit;
        int32_t           glContextMajor;
        int32_t           glContextMinor;
        SDL_GLcontextFlag glContextFlags;
        SDL_GLprofile     glProfileMask;
        bool              glDoubleBuffer;

        explicit Settings(
            const std::string& windowName = "",
            int64_t windowWidth = 800,
            int64_t windowHeight = 600,
            int64_t windowFramerateLimit = 60,
            int32_t glContextMajor = 3,
            int32_t glContextMinor = 3,
            SDL_GLcontextFlag glContextFlags = SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG,
            SDL_GLprofile glProfileMask = SDL_GL_CONTEXT_PROFILE_CORE,
            bool glDoubleBuffer = true) :
            windowName             (windowName),
            windowWidth            (windowWidth),
            windowHeight           (windowHeight),
            windowFramerateLimit   (windowFramerateLimit),
            glContextMajor(glContextMajor),
            glContextMinor(glContextMinor),
            glContextFlags(glContextFlags),
            glProfileMask(glProfileMask),
            glDoubleBuffer(glDoubleBuffer)
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

    // Window event handling loop
    void handleEvents(SDL_Event& event);
    void render(void);
};


#endif // SDL_APP_HPPs
