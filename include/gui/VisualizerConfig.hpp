#ifndef ROBOKASIV2_HOST_GUI_VISUALIZERCONFIG_HPP
#define ROBOKASIV2_HOST_GUI_VISUALIZERCONFIG_HPP

#include "gui/Puma560Model.hpp"

#include <array>
#include <functional>
#include <vector>
#include <string>

namespace gui {
    class VisualizerConfig {
    public:
        void render(std::shared_ptr<gui::Puma560Model> model);
        size_t registerSource(std::string name,
                              std::function<std::array<float, 6>(void)> fn);
        void setSource(size_t id);
        bool meshRenderEnable = true;
        bool frameRenderEnable = true;
    private:
        struct _VisualizerSource {
            std::string name;
            std::function<std::array<float, 6>(void)> fn;
        };
        std::vector<_VisualizerSource> _sources;
        size_t _curSourceId = 0;
    };
}

#endif
