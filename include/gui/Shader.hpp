//
// Created by lehdari on 20.10.2018.
//

#ifndef ROBOKASIV2_HOST_SHADER_HPP
#define ROBOKASIV2_HOST_SHADER_HPP


#include "GLTypes.hpp"
#include <unordered_map>


namespace gui {

    class Shader {
    public:
        Shader();
        ~Shader();
        void load(const std::string& vsFileName, const std::string& fsFileName);

        void addUniform(const std::string& name);

        void setUniform(const std::string& name, float uniform) const;
        void setUniform(const std::string& name, const Vec3GLf& uniform) const;
        void setUniform(const std::string& name, const Vec4GLf& uniform) const;
        void setUniform(const std::string& name, const Mat4GLf& uniform) const;

        void use() const;

    private:
        GLuint                                  _programId;
        std::unordered_map<std::string, GLint>  _uniformPositions;
    };

} // namespace gui


#endif //ROBOKASIV2_HOST_SHADER_HPP
