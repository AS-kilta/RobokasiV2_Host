//
// Created by lehdari on 14.10.2018.
//

#ifndef ROBOKASIV2_GUI_HOST_MESH_HPP
#define ROBOKASIV2_GUI_HOST_MESH_HPP


#include "GLTypes.hpp"


namespace gui {

    class Shader;
    class Camera;

    class Mesh {
    public:
        Mesh(void);
        ~Mesh(void);

        Mesh(const Mesh& other)             = delete;
        Mesh(Mesh&& other);
        Mesh& operator=(const Mesh& other)  = delete;
        Mesh& operator=(Mesh&& other)       = delete;

        void loadFromObj(const std::string& fileName);
        void render(const Shader& shader,
                    const Camera& camera,
                    const Vec3GLf& color = Vec3GLf(1.0f, 1.0f, 1.0f)) const;

        void setPosition(const Vec3GLf& position);
        void setRotation(const Mat3GLf& rotation);

        Mat4GLf getOrientation(void) const;

    private:
        GLuint _vertexArrayObjectId;
        unsigned _nIndices;

        GLuint _positionBufferId;
        GLuint _normalBufferId;
        GLuint _elementBufferId;

        Vec3GLf _position;
        Mat3GLf _rotation;
    };

} // namespace gui


#endif //ROBOKASIV2_GUI_HOST_MESH_HPP
