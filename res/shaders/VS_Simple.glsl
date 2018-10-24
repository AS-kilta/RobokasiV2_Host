/**

    Automaatio- ja Systeemitekniikan killan robokäsiprojekti
    VS_Simple.glsl

    @version    0.1
    @author     Miika 'LehdaRi' Lehtimäki
    @date       2015-04-18

**/


#version 330

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

out vec4 pos;
out vec3 norm;
out vec3 col;

uniform mat4 MVP;
uniform vec3 Color;

void main() {
    pos = MVP * vec4(position, 1.0);
    norm = (MVP * vec4(normal, 0.0)).xyz;
    col = Color;
    gl_Position = pos;
}
