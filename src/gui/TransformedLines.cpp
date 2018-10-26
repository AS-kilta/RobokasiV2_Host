#include "TransformedLines.hpp"

using namespace gui;

TransformedLines::TransformedLines(const std::shared_ptr<Lines>& lines,
                                   const Mat4GLf objectToWorld) :
    Renderable(objectToWorld),
    _lines(lines)
{ }


void TransformedLines::render(Shader& shader, Camera& camera)
{
    _lines->render(shader, camera, _objectToWorld);
}
