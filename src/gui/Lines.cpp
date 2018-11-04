#include "Lines.hpp"

#include "Camera.hpp"
#include "Shader.hpp"

using namespace gui;

namespace {
    std::vector<std::array<Vec3GLf, 2>> coordinateFrameEndpoints = {
        { Vec3GLf(0.f, 0.f, 0.f),
          Vec3GLf(8.f, 0.f, 0.f) },
        { Vec3GLf(0.f, 0.f, 0.f),
          Vec3GLf(0.f, 8.f, 0.f) },
        { Vec3GLf(0.f, 0.f, 0.f),
          Vec3GLf(0.f, 0.f, 8.f) } };
    std::vector<std::array<Vec3GLf, 2>> coordinateFrameColors = {
        { Vec3GLf(1.f, 0.f, 0.f),
          Vec3GLf(1.f, 0.f, 0.f) },
        { Vec3GLf(0.f, 1.f, 0.f),
          Vec3GLf(0.f, 1.f, 0.f) },
        { Vec3GLf(0.f, 0.f, 1.f),
          Vec3GLf(0.f, 0.f, 1.f) } };
}

Lines::Lines(std::vector<std::array<Vec3GLf, 2>> endpoints, std::vector<std::array<Vec3GLf, 2>> colors)
{
    if (endpoints.size() != colors.size()) {
        printf("Lines: Segment count doesn't match number of color pairs\n");
        throw;
    }
    // Generate buffers
    glGenVertexArrays(1, &_vertexArrayObjectId);
    glGenBuffers(1, &_positionBufferId);
    glGenBuffers(1, &_colorBufferId);


    // Bind VAO
    glBindVertexArray(_vertexArrayObjectId);

    // Upload segment data to gpu and set vertex attributes
    glBindBuffer(GL_ARRAY_BUFFER, _positionBufferId);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3GLf) * endpoints.size() * 2, &endpoints[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    glBindBuffer(GL_ARRAY_BUFFER, _colorBufferId);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vec3GLf) * colors.size() * 2, &colors[0], GL_STATIC_DRAW);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (GLvoid*)0);

    // Unbind VAO for safety
    glBindVertexArray(0);

    _nLines = endpoints.size() * 2;
}

std::shared_ptr<Lines> Lines::CoordinateFrame() {
    return std::make_shared<Lines>(coordinateFrameEndpoints, coordinateFrameColors);
}

Lines::~Lines(void)
{
    glDeleteVertexArrays(1, &_vertexArrayObjectId);
    glDeleteBuffers(1, &_positionBufferId);
    glDeleteBuffers(1, &_colorBufferId);
}

Lines::Lines(Lines&& other) :
    _vertexArrayObjectId(other._vertexArrayObjectId),
    _positionBufferId(other._positionBufferId),
    _colorBufferId(other._colorBufferId),
    _nLines(other._nLines)
{
    other._vertexArrayObjectId = 0;
    other._positionBufferId = 0;
    other._colorBufferId = 0;
    other._nLines = 0;
}

void Lines::render(
    const Shader& shader,
    const Camera& camera,
    const Mat4GLf& modelToWorld) const
{
    shader.use();
    shader.setUniform("modelToClip", Mat4GLf(camera.getWorldToClip() * modelToWorld));

    glBindVertexArray(_vertexArrayObjectId);

    glDrawArrays(GL_LINES, 0, _nLines * 2);

    glBindVertexArray(0);
}
