#ifndef ROBOKASIV2_HOST_GUI_LINES_HPP
#define ROBOKASIV2_HOST_GUI_LINES_HPP

#include <array>
#include <memory>
#include <vector>

#include "GLTypes.hpp"

namespace gui {

    class Shader;
    class Camera;

    class Lines {
    public:
        Lines(std::vector<std::array<Vec3GLf, 2>> endpoints,
              std::vector<std::array<Vec3GLf, 2>> colors);
        static std::shared_ptr<Lines> CoordinateFrame();
        ~Lines(void);

        Lines(const Lines& other)             = delete;
        Lines(Lines&& other);
        Lines& operator=(const Lines& other)  = delete;
        Lines& operator=(Lines&& other)       = delete;

        void render(const Shader& shader,
                    const Camera& camera,
                    const Mat4GLf& modelToWorld) const;

    private:

        GLuint _vertexArrayObjectId;
        GLuint _positionBufferId;
        GLuint _colorBufferId;
        GLuint _nLines;
    };

} // namespace gui

#endif //ROBOKASIV2_HOST_GUI_LINES_HPP
