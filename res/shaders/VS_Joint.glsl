/**

    Automaatio- ja Systeemitekniikan killan robok�siprojekti
    VS_Joint.glsl

    @version    0.1
    @author     Miika 'LehdaRi' Lehtim�ki
    @date       2015-04-18

**/


#version 330

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;

out vec3 col;

uniform mat4 MVP;
uniform vec3 Color;

void main() {
    col = color;
    gl_Position = MVP * position;
}
