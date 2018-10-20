//
// Created by lehdari on 20.10.2018.
//

#ifndef ROBOKASIV2_HOST_SHADER_HPP
#define ROBOKASIV2_HOST_SHADER_HPP


#include "GLTypes.hpp"


namespace gui {

    class Shader {
    public:
        Shader();
        ~Shader();
        void load(const std::string& vsFileName, const std::string& fsFileName);

        void useShader(const Mat4GLf& mvp,
                       const Vec3GLf& color = Vec3GLf(1.0f, 1.0f, 1.0f)) const;

    private:
        GLuint programId_;
        GLuint uniformPosition_MVP_;
        GLuint uniformPosition_Color_;
    };

} // namespace gui


#endif //ROBOKASIV2_HOST_SHADER_HPP
