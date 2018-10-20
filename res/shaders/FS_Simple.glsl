/**

    Automaatio- ja Systeemitekniikan killan robok�siprojekti
    FS_Simple.glsl

    @version    0.1
    @author     Miika 'LehdaRi' Lehtim�ki
    @date       2015-04-18

**/


#version 130

in vec4 pos;
in vec4 norm;
in vec3 col;

out vec3 color;

void main() {
    color = col * (0.35+clamp(dot(normalize(vec3(-1.0f, -1.0f, -1.0f)), normalize(norm.xyz)), 0.0, 1.0)*0.65);
}
