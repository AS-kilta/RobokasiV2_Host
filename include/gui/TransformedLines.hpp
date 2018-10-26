#ifndef ROBOKASIV2_HOST_GUI_TRANSFORMEDLINES_HPP
#define ROBOKASIV2_HOST_GUI_TRANSFORMEDLINES_HPP

#include <memory>

#include "GLTypes.hpp"
#include "Lines.hpp"
#include "Renderable.hpp"

namespace gui {

    class TransformedLines : public Renderable {
    public:
        TransformedLines(const std::shared_ptr<Lines>& lines,
                         const Mat4GLf objectToWorld = Mat4GLf::Identity());

        void render(Shader& shader, Camera& camera);

    private:
        std::shared_ptr<Lines> _lines;
    };

} // namespace gui


#endif //ROBOKASIV2_HOST_GUI_TRANSFORMEDLINES_HPP
